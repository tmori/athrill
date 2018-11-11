#include "logger.h"

using namespace athrill::ros::lib;


int main(int argc, const char* argv[])
{
    LoggerConfigType config;

    config.max_message_size_each_line = 4096;
    config.multi_process_sharing = true;
    config.multi_thread_sharing = true;
    config.save_as_all_level_one_file = true;
    config.logfile.max_filesize = 1;
    config.logfile.backup_num = 3;
    config.logfile.prefix_logname = "test_node";
    config.logfile.dest_folder = "./test/log";
    Logger logger = Logger(config);

    logger.init();
    logger.log(LOG_INFO, "this is a test log(%d)", 1);
    logger.log(LOG_ERROR, "this is a error log(%d)", 123);
    logger.log(LOG_WARN, "this is a warning log(%s)", "abc");
    logger.log(LOG_DEBUG, "this is a warning log(%s)", "debug");

    return 0;
}