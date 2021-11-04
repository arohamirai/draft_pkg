
#include <string>
#include <iostream>
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <map>
namespace fs = boost::filesystem;
using namespace std;
int  main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;

    string root_path = string(getenv("HOME")) + "/camera_servo_record";
    if(!fs::exists(root_path))
    {
        LOG(INFO) <<root_path << " not exist, create it.";
        CHECK(fs::create_directory(root_path)) << "Create FileFolder Failed: " <<root_path;
    }
    else
        LOG(INFO) <<root_path << " exist, skip make it." << endl;

      // totay
        struct tm *ptm;
        long ts;
        int y, m, d;

        ts = time(NULL);
        ptm = localtime(&ts);
        y = ptm->tm_year + 1900;
        m = ptm->tm_mon + 1;
        d = ptm->tm_mday;

    string today = root_path+ "/" +std::to_string(y) + std::to_string(m) + std::to_string(d);
    if(!fs::exists(today))
    {
        LOG(INFO) << today << " not exist, create it.";
        CHECK(fs::create_directory(today)) << "Create FileFolder Failed: " <<today;
    }
    else
        LOG(INFO) <<today << " exist, skip make it." << endl;

    // how many folders
    typedef std::multimap<std::time_t, fs::path, std::less<std::time_t>> result_set_t;
    result_set_t folders;
    //std::vector<FolderInfo> folders;
    fs::directory_iterator item(root_path);
    fs::directory_iterator item_end;
    for( ;item  != item_end; item++)
    {
        LOG(INFO) << "item: " << *item;
        if(fs::is_directory(item->path()))
        {
            std::time_t  time = fs::last_write_time(item->path());
            folders.insert(result_set_t::value_type(time, item->path()));
        }
    }
    LOG(INFO) << "==================after soft============";
    for(auto it:folders)
    {
        fs::path path = it.second;
        LOG(INFO) << path.string();
    }

    int folder_num = folders.size();
    if(folder_num > 10)
    {
        LOG(INFO) << "folder_num: " << folder_num <<">10, remove older folder.";
        for(auto it:folders)
        {
            fs::path path = it.second;
            fs::remove_all(path);
            LOG(INFO) << path.string() << " has been removed.";
            folder_num--;
            if(folder_num <=10)
            break;
        }
    }

    fs::directory_iterator today_file(today);
    fs::directory_iterator today_end;
    int today_file_num = 0;
    for( ;today_file != today_end; today_file++)
    {
        if(fs::is_directory(today_file->path()))
            continue;
        LOG(INFO) << "today_file: " << *today_file;
        today_file_num++;
    }
    LOG(INFO) << "today_file_num: " << today_file_num;
return 0;
}