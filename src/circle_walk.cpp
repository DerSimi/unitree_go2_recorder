#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unistd.h>
#include <cmath>
#include <thread>

// radius radius in m
// vx x velociy in m/s
// circle amount = 0.5 would be half a circle, etc.
void walkCircle(unitree::robot::go2::SportClient& client,
                float radius,
                float vx,
                float circle_amount = 1.0f)
{
    float vyaw = vx / radius;             
    float duration = 2.0f * M_PI * radius / std::abs(vx) * 1.37f * circle_amount;

    // timestamp at start
    auto t0 = std::chrono::steady_clock::now();

    while (std::chrono::duration<float>(
               std::chrono::steady_clock::now() - t0)
               .count() < duration)
    {
        client.Move(vx, 0.0f, vyaw);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    client.Move(0.0f, 0.0f, 0.0f);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
//argv[1] is network interface of the robot
  
  //Create a sports client object
  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);//Timeout time
  sport_client.Init();

  sport_client.StandUp(); //Special action, robot dog sitting down
  sleep(3);//delay 3s

  sport_client.BalanceStand();
  sleep(1);

  float vx_seed = 0.6f;

  sport_client.SwitchGait(1);
  walkCircle(sport_client, 1.5, vx_seed, 0.4);

  sport_client.SwitchGait(2);
  walkCircle(sport_client, 1.5, vx_seed, 0.4);

  sport_client.SwitchGait(3);
  walkCircle(sport_client, 1.5, vx_seed, 0.4);

  sport_client.SwitchGait(4);
  walkCircle(sport_client, 1.5, -vx_seed, 0.4);

  sport_client.SwitchGait(2);
  walkCircle(sport_client, 1.5, -vx_seed, 0.4);

  sport_client.SwitchGait(1);

  sport_client.StandDown(); //Restore
  return 0;
}