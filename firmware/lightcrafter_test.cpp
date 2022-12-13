#include <iostream>
#include "easylogging++.h"
#include "lightcrafter3010.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream> 
#include <unistd.h> 
#include "protocol.h"
#include <random>
#include <time.h> 
#include <thread>

INITIALIZE_EASYLOGGINGPP


int main()
{
    LOG(INFO)<<"lightcrafter test:";

    LightCrafter3010 lc3010_;

    lc3010_.init();  
    LOG(INFO)<<"lc3010 init";

    int num = 1000000;

    while (num-- > 0)
    {
        lc3010_.pattern_mode04();

        // LOG(INFO) << "pattern_mode04";
        lc3010_.start_pattern_sequence();
        // LOG(INFO) << "pattern_mode04";


        // lc3010_.enable_solid_field();
        // LOG(INFO) << "enable_solid_field";
        // usleep(10000);
        // lc3010_.disable_solid_field();
        // LOG(INFO) << "disable_solid_field";

        lc3010_.SetLedCurrent(512, 512, 512);
        LOG(INFO) << "SetLedCurrent: "
                  << "512";
        // lc3010_.enable_solid_field();
        // LOG(INFO) << "enable_solid_field";
        // usleep(10000);
        // lc3010_.disable_solid_field();
        // LOG(INFO) << "disable_solid_field";

        lc3010_.SetLedCurrent(1023, 1023, 1023);
        LOG(INFO) << "SetLedCurrent: "
                  << "1023";

        lc3010_.SetLedCurrent(255, 255, 255);
        LOG(INFO) << "SetLedCurrent: "
                  << "255";

        // lc3010_.enable_solid_field();
        // LOG(INFO) << "enable_solid_field";
        // usleep(10000);
        // lc3010_.disable_solid_field();
        // LOG(INFO) << "disable_solid_field";
        sleep(1);
        LOG(INFO) << "num: " << num;
    }

    return 0;

}