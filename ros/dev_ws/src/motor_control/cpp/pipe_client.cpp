#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

int main()
{
    const char *pipe_name = "/tmp/steer_throttle_pipe";

    std::ofstream pipe(pipe_name);

    if (!pipe.is_open())
    {
        std::cerr << "Failed to open pipe: " << pipe_name << std::endl;
        return 1;
    }

    float steer = 90;
    float throttle = 0;

    pipe << steer << "," << throttle << std::endl;
    pipe.flush();

    std::cout << "Sent steer: " << steer << ", throttle: " << throttle << std::endl;

    return 0;

    while (true)
    {
        steer += 1;
        throttle += 0.05;

        if (steer > 120)
            steer = 60;

        if (throttle > 1)
            throttle = -1;

        pipe << steer << "," << throttle << std::endl;
        pipe.flush();

        std::cout << "Sent steer: " << steer << ", throttle: " << throttle << std::endl;

        usleep(50 * 1000); // 50 ms 20 Hz
    }

    pipe.close();
    return 0;
}
