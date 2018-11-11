#include "logger.h"

using namespace athrill::ros::lib;


int main(int argc, const char* argv[])
{
    LoggerConfigType config;
    LoggerConfigType config_1;

    config.log_level = LOG_ALL;
    config.max_filesize = 1;
    config.max_message_size_each_line = 4096;
    config.multi_process_sharing = true;
    config.multi_thread_sharing = true;
    config.logfile.backup_num = 3;
    config.logfile.prefix_logname = "test_node";
    config.logfile.dest_folder = "./test/log";
    Logger logger = Logger(config);

    config_1.log_level = LOG_ERROR;
    config_1.max_filesize = 1;
    config_1.max_message_size_each_line = 4096;
    config_1.multi_process_sharing = true;
    config_1.multi_thread_sharing = true;
    config_1.logfile.backup_num = 2;
    config_1.logfile.prefix_logname = "test_node_error";
    config_1.logfile.dest_folder = "./test/log_error";
    Logger logger_1 = Logger(config_1);

    logger.init();
    logger.log(LOG_INFO, "this is a test log(%d)", 1);

    logger_1.init();
    logger_1.log(LOG_ERROR, "this ERROR");
    return 0;
}