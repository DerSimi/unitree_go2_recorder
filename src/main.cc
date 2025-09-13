// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>

#include <mujoco/mujoco.h>
#include <pthread.h>

#include "mujoco/glfw_adapter.h"
#include "mujoco/simulate.h"
#include "mujoco/array_safety.h"
#include "extractor/extractor.h"
#include "storage/storage_handler.h"

extern "C"
{
#include <sys/errno.h>
#include <unistd.h>
}

namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;
  helperData *helper_data = nullptr; // For collecting more data

  // NEU: Extractor-Instanz, die von beiden Threads genutzt wird
  std::shared_ptr<MujocoExtractor> unitree_interface;

  using Seconds = std::chrono::duration<double>;

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim, StorageHandler &handler)
  {
    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      // Prüfen, ob der Extractor initialisiert ist
      if (unitree_interface)
      {
        double timestamp;
        // Versuche, einen neuen, synchronisierten Zustand zu bekommen
        if (unitree_interface->GetSynchronizedState(timestamp))
        {
          const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

          handler.addState(d, helper_data, timestamp);

          mj_forward(m, d);

          sim.speed_changed = true;
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)

  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      sim->Load(m, d, filename);

      // Set camera to track the robot's base_link, comment out this block to disable camera tracking
      int body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
      if (body_id != -1)
      {
        sim->cam.type = mjCAMERA_TRACKING;
        sim->cam.trackbodyid = body_id;
        sim->cam.distance = 3.0;
      }

      mj_forward(m, d);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  StorageHandler storage_handler(m->nq, m->nv, m->nu);

  PhysicsLoop(*sim, storage_handler);

  cout << "Physics thread exiting..." << endl;

  // store data
  storage_handler.storeData();

  // delete everything we allocated
  mj_deleteData(d);
  mj_deleteModel(m);

  exit(0);
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  // Wait for mujoco data
  while (1)
  {
    if (d)
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);
  }

  helper_data = new helperData(m->nu);

  //Init ros
  rclcpp::init(0, nullptr);

  // Change queue for maximal performance (timing)
  rclcpp::QoS qos(1); // Queue 1
  qos.best_effort();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 12);
  
  unitree_interface = std::make_shared<MujocoExtractor>(m, d, helper_data);
  executor.add_node(unitree_interface);
  executor.spin();

  delete helper_data;

  rclcpp::shutdown();

  pthread_exit(NULL);
}
//------------------------------------------ main --------------------------------------------------

// run event loop
int main(int argc, char **argv)
{
  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false);

  const char *filename = "../model/scene.xml";

  pthread_t unitree_thread;
  int rc = pthread_create(&unitree_thread, NULL, UnitreeSdk2BridgeThread, NULL);
  if (rc != 0)
  {
    std::cout << "Error:unable to create thread," << rc << std::endl;
    exit(-1);
  }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return 0;
}
