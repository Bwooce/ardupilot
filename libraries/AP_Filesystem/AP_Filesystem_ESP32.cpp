/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  ArduPilot filesystem interface for esp32 systems
 */
#include "AP_Filesystem.h"
#include <AP_HAL/AP_HAL.h>

#if AP_FILESYSTEM_ESP32_ENABLED

#define FSDEBUG 0  // Disable verbose filesystem debug to prevent logging overflow

#include <utime.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "esp_log.h"


#ifdef HAL_ESP32_USE_SPIFFS
#include "esp_spiffs.h"
#endif

// Removed unused TAG variable

extern const AP_HAL::HAL& hal;

// Constructor to verify object creation
AP_Filesystem_ESP32::AP_Filesystem_ESP32()
{
    // Logging removed to prevent stack overflow
}

// Destructor
AP_Filesystem_ESP32::~AP_Filesystem_ESP32()
{
    // Logging removed to prevent stack overflow
}

int AP_Filesystem_ESP32::open(const char *fname, int flags, bool allow_absolute_paths)
{
    // Minimal logging to avoid stack overflow
#if FSDEBUG
    ESP_LOGI(TAG, "open %s flags=0x%x", fname, flags);
#endif

#if FSDEBUG
    ESP_LOGI(TAG, "open %s flags=0x%x (O_CREAT=%d, O_TRUNC=%d, O_WRONLY=%d)",
             fname, flags,
             (flags & O_CREAT) ? 1 : 0,
             (flags & O_TRUNC) ? 1 : 0,
             (flags & O_WRONLY) ? 1 : 0);

    // Check if file exists before trying to create
    struct stat st;
    if (stat(fname, &st) == 0) {
        ESP_LOGI(TAG, "File %s exists, size=%ld", fname, st.st_size);
    } else {
        ESP_LOGI(TAG, "File %s does not exist, will create", fname);
    }
#endif

    // we automatically add O_CLOEXEC as we always want it for ArduPilot FS usage
    // Note: Don't add O_TRUNC here - let the caller specify it if needed


#ifdef HAL_ESP32_USE_SPIFFS
    // SPIFFS doesn't support O_CLOEXEC, remove it if present
    int spiffs_flags = flags & ~O_CLOEXEC;

    int fd;
    if (spiffs_flags & O_CREAT) {
        // For file creation, MUST provide mode
        fd = ::open(fname, spiffs_flags, 0666);
    } else {
        // For opening existing files, mode is not needed
        fd = ::open(fname, spiffs_flags);
    }
#else
    int fd = ::open(fname, flags | O_CLOEXEC, 0666);
#endif


#if FSDEBUG
    if (fd < 0) {
        int saved_errno = errno;
        ESP_LOGE(TAG, "Open failed for %s: %s (errno=%d)", fname, strerror(saved_errno), saved_errno);

        // Additional diagnostics for ENOSPC
        if (saved_errno == ENOSPC || saved_errno == 28) {  // 28 is ENOSPC
            ESP_LOGE(TAG, "ENOSPC error - checking filesystem state");

            // Try to create a simple test file
            int test_fd = ::open("/APM/test.tmp", O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
            if (test_fd >= 0) {
                ESP_LOGW(TAG, "Test file creation succeeded! Issue may be with specific filename");
                ::close(test_fd);
                ::unlink("/APM/test.tmp");
            } else {
                ESP_LOGE(TAG, "Test file creation also failed: %s", strerror(errno));
            }

            // Check how many files exist
            DIR* d = ::opendir("/APM");
            if (d != nullptr) {
                int count = 0;
                while (::readdir(d) != nullptr) {
                    count++;
                }
                ::closedir(d);
                ESP_LOGI(TAG, "Current file count in /APM: %d (max_files=50)", count);
            }
        }
    } else {
        ESP_LOGI(TAG, "Open success fd=%d for %s", fd, fname);
    }
#endif
    return fd;
}

int AP_Filesystem_ESP32::close(int fd)
{
#if FSDEBUG
    printf("DO close \n");
#endif
    return ::close(fd);
}

int32_t AP_Filesystem_ESP32::read(int fd, void *buf, uint32_t count)
{
#if FSDEBUG
    printf("DO read \n");
#endif
    return ::read(fd, buf, count);
}

int32_t AP_Filesystem_ESP32::write(int fd, const void *buf, uint32_t count)
{
#if FSDEBUG
    printf("DO write \n");
#endif
    return ::write(fd, buf, count);
}

int AP_Filesystem_ESP32::fsync(int fd)
{
#if FSDEBUG
    printf("DO fsync \n");
#endif
    return ::fsync(fd);
}

int32_t AP_Filesystem_ESP32::lseek(int fd, int32_t offset, int seek_from)
{
#if FSDEBUG
    printf("DO lseek \n");
#endif
    return ::lseek(fd, offset, seek_from);
}

int AP_Filesystem_ESP32::stat(const char *pathname, struct stat *stbuf)
{
#if FSDEBUG
    printf("DO stat %s \n", pathname);
#endif
    return ::stat(pathname, stbuf);
}

int AP_Filesystem_ESP32::unlink(const char *pathname)
{
#if FSDEBUG
    printf("DO unlink %s \n", pathname);
#endif
    return ::unlink(pathname);
}

int AP_Filesystem_ESP32::rename(const char *oldpath, const char *newpath)
{
#if FSDEBUG
    printf("DO rename %s -> %s\n", oldpath, newpath);
#endif
    return ::rename(oldpath, newpath);
}

int AP_Filesystem_ESP32::mkdir(const char *pathname)
{
#if FSDEBUG
    printf("DO mkdir %s \n", pathname);
#endif
    return ::mkdir(pathname, 0777);
}

void* AP_Filesystem_ESP32::opendir(const char *pathname)
{
#if FSDEBUG
    printf("DO opendir %s \n", pathname);
#endif

    return (void*)::opendir(pathname);
    //	return NULL;
}

struct dirent *AP_Filesystem_ESP32::readdir(void *dirp)
{
#if FSDEBUG
    printf("DO readdir \n");
#endif
    return ::readdir((DIR*)dirp);
    //	return NULL;
}

int AP_Filesystem_ESP32::closedir(void *dirp)
{
#if FSDEBUG
    printf("DO closedir \n");
#endif

    return ::closedir((DIR*)dirp);
    //	return 0;
}

// return number of bytes that should be written before fsync for optimal
// streaming performance/robustness. if zero, any number can be written.
// assume that ESP32 is similar to FAT and 4K boundaries are good.
uint32_t AP_Filesystem_ESP32::bytes_until_fsync(int fd)
{
    int32_t pos = ::lseek(fd, 0, SEEK_CUR);
    if (pos < 0) {
        return 0;
    }

    const uint32_t block_size = 4096;

    uint32_t block_pos = (uint32_t)pos % block_size;
    return block_size - block_pos;
}

// return free disk space in bytes
int64_t AP_Filesystem_ESP32::disk_free(const char *path)
{
#if FSDEBUG
    ESP_LOGI(TAG, "disk_free for path: %s", path);
#endif

#ifdef HAL_ESP32_USE_SPIFFS
    // For SPIFFS, use esp_spiffs_info to get free space
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info("logs", &total, &used);
    if (ret != ESP_OK) {
        // Try without partition label (for older ESP-IDF) - logging removed to prevent stack overflow
        ret = esp_spiffs_info(NULL, &total, &used);
        if (ret != ESP_OK) {
            // Error logging removed to prevent stack overflow
            return -1;
        }
    }
    int64_t free_bytes = (int64_t)(total - used);
    // Logging removed to prevent stack overflow during filesystem operations
    return free_bytes;
#else
    // Original FATFS code for SD card
    FATFS *fs;
    DWORD fre_clust, fre_sect;

    /* Get volume information and free clusters of sdcard */
    auto res = f_getfree("/SDCARD/", &fre_clust, &fs);
    if (res) {
        return -1;
    }

    /* Get total sectors and free sectors */
    fre_sect = fre_clust * fs->csize;

    int64_t tmp_free_bytes = fre_sect * FF_SS_SDCARD;

    return tmp_free_bytes;
#endif
}

// return total disk space in bytes
int64_t AP_Filesystem_ESP32::disk_space(const char *path)
{
#if FSDEBUG
    printf("DO usage disk %s \n", path);
#endif

#ifdef HAL_ESP32_USE_SPIFFS
    // For SPIFFS, use esp_spiffs_info to get total space
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info("logs", &total, &used);
    if (ret != ESP_OK) {
        // Try without partition label (for older ESP-IDF)
        ret = esp_spiffs_info(NULL, &total, &used);
        if (ret != ESP_OK) {
            return -1;
        }
    }
    return (int64_t)total;
#else
    // Original FATFS code for SD card
    FATFS *fs;
    DWORD fre_clust, tot_sect;

    /* Get volume information and free clusters of sdcard */
    auto res = f_getfree("/SDCARD/", &fre_clust, &fs);
    if (res) {
        return -1;
    }

    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;

    int64_t tmp_total_bytes = tot_sect * FF_SS_SDCARD;

    return tmp_total_bytes;
#endif
}

/*
  set mtime on a file
 */
bool AP_Filesystem_ESP32::set_mtime(const char *filename, const uint32_t mtime_sec)
{

#if FSDEBUG
    printf("DO time %s \n", filename);
#endif
    struct utimbuf times {};
    times.actime = mtime_sec;
    times.modtime = mtime_sec;

    return utime(filename, &times) == 0;
}

#endif  // AP_FILESYSTEM_ESP32_ENABLED
