void WriteFormattedTimestamp(std::string header, int64 utime)
{
    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::localtime(&seconds);
    std::cout << header << ": ";
    std::cout << std::put_time(t, "%Y-%m-%d %H:%M:%S");
    printf(".%03d\n", milliseconds);
}

std::string GetFormattedTimestamp(int64 utime)
{
    std::string output;
    std::stringstream ss;

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::localtime(&seconds);

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

std::string GetFormattedTimestampCurrent()
{
    std::string output;
    std::stringstream ss;

    int64 utime = GetCurrentMicroseconds();

    std::time_t seconds = utime / 1000000;
    int milliseconds = (utime % 1000000) / 1000;
    std::tm* t = std::localtime(&seconds);

    ss << std::put_time(t, "%Y-%m-%d_%H-%M-%S");
    ss << boost::format("-%03d") % milliseconds;
    output = ss.str();

    return output;
}

long int GetCurrentMicroseconds()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int us = tp.tv_sec * 1000000 + tp.tv_usec;
    return us;
}

long int GetCurrentMicroseconds()
{
    using namespace std::chrono;
    milliseconds ms = duration_cast< milliseconds >(
        system_clock::now().time_since_epoch()
    );
}

int mkdir_recursive(const char* file_path_, mode_t mode) {
    assert(file_path_ && *file_path_);
    char file_path[strlen(file_path_)];
    strcpy(file_path, file_path_);
    char * pfile_path = file_path;
    char* p;
    for (p=strchr(pfile_path+1, '/'); p; p=strchr(p+1, '/')) {
        *p='\0';
        if (mkdir(pfile_path, mode)==-1) {
            if (errno!=EEXIST) { *p='/'; return -1; }
        }
        *p='/';
    }
    return 0;
}

std::vector<std::string> ListOfFilesPattern(std::string path, std::string pattern)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    std::vector<std::string> files;

    if ((dir = opendir (path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                if (strstr(ent->d_name, pattern.c_str()) != NULL) {
                    files.push_back(std::string(ent->d_name));
                }
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", path.c_str());
    }

    std::sort(std::begin(files), std::end(files));

    return files;
}

std::vector<std::string> ListOfFiles(std::string path)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    std::vector<std::string> files;

    if ((dir = opendir (path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                files.push_back(std::string(ent->d_name));
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", path.c_str());
    }

    std::sort(std::begin(files), std::end(files));

    return files;
}

std::string FindFileFromPath(std::string filenameSubpart, std::string searchPath)
{
    DIR *dir;
    struct dirent *ent;
    std::string output;
    if ((dir = opendir (searchPath.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) {
                if (strstr(ent->d_name, filenameSubpart.c_str()) != NULL) {
                    output = searchPath.substr(0, searchPath.find_last_of("/")) + "/" + std::string(ent->d_name);
                    return output;
                }
            }
        }
        closedir (dir);
    } else { // could not open directory
        printf("Error: Could not open directory: %s\n", searchPath.c_str());
    }

    return output;
}

std::string SystemCall(std::string cmd) {

    string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        pclose(stream);
    }
    return data;
}

inline bool PathExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0);
}

inline bool FileExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0 && S_ISREG(sb.st_mode));
}

inline bool FolderExist(const std::string& path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)); // folder exists
}
