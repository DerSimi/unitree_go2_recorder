// Code is heavily edited, but in the original file, the following license applies:
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

#include "extractor/simulator.hpp"

Simulator *Simulator::instance_ = nullptr;

void Simulator::sig_handler(int signum)
{
    if (instance_)
    {
        instance_->terminate();
    }
}

Simulator::Simulator(ExtractorMode mode, std::string storage_path, std::string model_path)
{
    instance_ = this;
    this->storage_path_ = storage_path;
    this->mode_ = mode;

    // print version, check compatibility
    spdlog::info("MuJoCo version {}", mj_versionString());
    if (mjVERSION_HEADER != mj_version())
    {
        spdlog::warn("MuJoCo headers and library have different versions");
    }

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    sim_ = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::thread extractor_handle = std::thread([this]()
                                               { this->extractor_thread(); });

    spdlog::info("Using model: {}", model_path);
    // Makes argument passing more convenient
    model_path = "../" + model_path;

    // start physics thread
    const char *model_path_cstr = model_path.c_str();
    std::thread physics_thread_handle([this, model_path_cstr]()
                                      { this->physics_thread(model_path_cstr); });
    // start simulation UI loop (blocking call)
    sim_->RenderLoop();

    // Terminate function will kill the blocking call above, then we join the threads, game over.
    physics_thread_handle.join();
    extractor_handle.join();
}

mjModel *Simulator::load_model(const char *file)
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
    char loadError[K_ERROR_LENGTH] = "";
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
        mnew = mj_loadXML(filename, nullptr, loadError, K_ERROR_LENGTH);
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

    mju::strcpy_arr(sim_->load_error, loadError);

    if (!mnew)
    {
        std::printf("%s\n", loadError);
        return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
        // mj_forward() below will print the warning message
        spdlog::warn("Model compiled, but simulation warning (paused):\n  {}", loadError);
        sim_->run = 0;
    }

    return mnew;
}

void Simulator::physics_loop()
{
    // run until asked to exit
    while (!sim_->exitrequest.load())
    {
        if (extractor_node_)
        { // lock sim mutex
            const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
            if (extractor_node_->get_rendering_state())
            {
                //get_rendering_state will overwrite d_.
                mj_forward(m_, d_);
                sim_->speed_changed = true;
            }
        } // unlock sim mutex

        // sleep for 1 ms or yield, to let main thread run
        if (sim_->run && sim_->busywait)
        {
            std::this_thread::yield();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Simulator::physics_thread(const char *file)
{
    // request loadmodel if file given (otherwise drag-and-drop)

    if (file != nullptr)
    {
        sim_->LoadMessage(file);
        m_ = load_model(file);
        if (m_)
            d_ = mj_makeData(m_);
        if (d_)
        {
            sim_->Load(m_, d_, file);

            // Set camera to track the robot's base_link, comment out this block to disable camera tracking
            int body_id = mj_name2id(m_, mjOBJ_BODY, "base_link");
            if (body_id != -1)
            {
                sim_->cam.type = mjCAMERA_TRACKING;
                sim_->cam.trackbodyid = body_id;
                sim_->cam.distance = 3.0;
            }

            mj_forward(m_, d_);
        }
        else
        {
            sim_->LoadMessageClear();
        }
    }

    physics_loop();

    // delete everything we allocated
    mj_deleteData(d_);
    mj_deleteModel(m_);

    terminate();
}

void Simulator::extractor_thread()
{
    // Wait for mujoco data
    while (1)
    {
        if (d_)
        {
            spdlog::info("Simulator is ready for data.");
            break;
        }
        usleep(500000);
    }

    // Init ros
    rclcpp::init(0, nullptr);

    // Override ros signal handler
    std::signal(SIGINT, Simulator::sig_handler);

    // Change queue for maximal performance (timing)
    rclcpp::QoS qos(1); // Queue 1
    qos.best_effort();

    storage_handler_ = std::make_unique<StorageHandler>(m_->nq, m_->nv, m_->nu);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    extractor_node_ = std::make_shared<MujocoExtractor>(m_, d_, this->mode_, storage_handler_.get());
    executor.add_node(extractor_node_);
    executor.spin();
}

void Simulator::terminate()
{
    if (instance_)
    {
        spdlog::info("Terminate extractor...");

        // Avoid calling terminate twice
        instance_ = nullptr;

        // Kill MuJoCo
        sim_->exitrequest.store(true);

        rclcpp::shutdown();

        // store data
        std::filesystem::path outdir = "../output";
        if (!std::filesystem::exists(outdir))
        {
            std::filesystem::create_directories(outdir);
        }

        storage_handler_->store_data((outdir / this->storage_path_).c_str());
    }
}