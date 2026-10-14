#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BAUD_RATE       230400U
#define FRAME_SIZE      1024U
#define FRAME_RATE      12U
#define READY_BYTE      0x7AU
#define LEGACY_READY    0x79U
#define ACK_BYTE        0x06U
#define FRAME_READY_BYTE 0x52U
#define BOARD_STOP_BYTE 0x18U

#define RESPONSE_ERROR      0
#define RESPONSE_ACK        1
#define RESPONSE_BOARD_STOP 2
#define RESPONSE_FRAME_READY 3

#define WAIT_ERROR        0
#define WAIT_READY        1
#define WAIT_OLD_FIRMWARE 2

static volatile LONG stop_requested = 0;

static void pause_before_close(int interactive_mode)
{
    if (interactive_mode != 0) {
        printf("Press Enter to close this window...");
        fflush(stdout);
        (void)getchar();
    }
}

static int find_default_video(char path[MAX_PATH])
{
    static const char frame_name[] = "BadApple_full_128x64_12fps.bin";
    char project_dir[MAX_PATH];
    char candidate[MAX_PATH];
    DWORD length;

    length = GetModuleFileNameA(NULL, project_dir, MAX_PATH);
    if (length == 0U || length >= MAX_PATH) {
        return 0;
    }

    /* executable file -> badapple -> Core -> the STM32 project directory */
    for (int level = 0; level < 3; ++level) {
        char *separator = strrchr(project_dir, '\\');
        if (separator == NULL) {
            return 0;
        }
        *separator = '\0';
    }

    if (snprintf(candidate, sizeof(candidate), "%s\\other\\%s",
                 project_dir, frame_name) <= 0 ||
        GetFileAttributesA(candidate) == INVALID_FILE_ATTRIBUTES) {
        return 0;
    }

    (void)snprintf(path, MAX_PATH, "%s", candidate);
    return 1;
}

static BOOL WINAPI console_handler(DWORD event)
{
    if (event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT) {
        InterlockedExchange(&stop_requested, 1L);
        return TRUE;
    }
    return FALSE;
}

static int write_all(HANDLE port, const uint8_t *data, DWORD size)
{
    DWORD total = 0U;

    while (total < size) {
        DWORD written = 0U;
        if (!WriteFile(port, data + total, size - total, &written, NULL)) {
            return 0;
        }
        total += written;
    }
    return 1;
}

static int read_one(HANDLE port, uint8_t expected, const char *description)
{
    uint8_t value = 0U;
    DWORD received = 0U;

    if (!ReadFile(port, &value, 1U, &received, NULL) || received != 1U) {
        fprintf(stderr, "Timeout waiting for %s.\n", description);
        return 0;
    }
    if (value != expected) {
        fprintf(stderr, "Wrong %s: received 0x%02X.\n", description, value);
        return 0;
    }
    return 1;
}

static int read_frame_response(HANDLE port)
{
    uint8_t value = 0U;
    DWORD received = 0U;

    if (!ReadFile(port, &value, 1U, &received, NULL) || received != 1U) {
        fprintf(stderr, "Timeout waiting for frame response.\n");
        return RESPONSE_ERROR;
    }
    if (value == ACK_BYTE) {
        return RESPONSE_ACK;
    }
    if (value == BOARD_STOP_BYTE) {
        return RESPONSE_BOARD_STOP;
    }
    if (value == FRAME_READY_BYTE) {
        return RESPONSE_FRAME_READY;
    }

    fprintf(stderr, "Wrong frame response: received 0x%02X.\n", value);
    return RESPONSE_ERROR;
}

static int wait_for_board(HANDLE port)
{
    static const uint8_t start_magic[4] = {'B', 'A', 'D', '!'};
    uint8_t value;
    DWORD received;

    while (InterlockedCompareExchange(&stop_requested, 0L, 0L) == 0L) {
        PurgeComm(port, PURGE_RXCLEAR);
        if (!write_all(port, start_magic, sizeof(start_magic))) {
            return 0;
        }
        received = 0U;
        if (ReadFile(port, &value, 1U, &received, NULL) &&
            received == 1U && value == READY_BYTE) {
            return WAIT_READY;
        }
        if (received == 1U && value == LEGACY_READY) {
            return WAIT_OLD_FIRMWARE;
        }
        printf("Still waiting: run the firmware and quickly press USER three times.\n");
    }
    return WAIT_ERROR;
}

static HANDLE open_port(const char *port_name)
{
    char device_path[64];
    HANDLE port;
    DCB dcb = {0};
    COMMTIMEOUTS timeouts = {0};

    snprintf(device_path, sizeof(device_path), "\\\\.\\%s", port_name);
    port = CreateFileA(device_path,
                       GENERIC_READ | GENERIC_WRITE,
                       0U,
                       NULL,
                       OPEN_EXISTING,
                       0U,
                       NULL);
    if (port == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Cannot open %s, Windows error %lu.\n",
                port_name, GetLastError());
        return INVALID_HANDLE_VALUE;
    }

    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(port, &dcb)) {
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = BAUD_RATE;
    dcb.ByteSize = 8U;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    if (!SetCommState(port, &dcb)) {
        fprintf(stderr, "Cannot configure %s.\n", port_name);
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }

    timeouts.ReadIntervalTimeout = 50U;
    timeouts.ReadTotalTimeoutConstant = 3000U;
    timeouts.ReadTotalTimeoutMultiplier = 0U;
    timeouts.WriteTotalTimeoutConstant = 3000U;
    timeouts.WriteTotalTimeoutMultiplier = 0U;
    SetCommTimeouts(port, &timeouts);
    SetupComm(port, 8192U, 8192U);
    PurgeComm(port, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return port;
}

int main(int argc, char **argv)
{
    static const uint8_t frame_header[2] = {0xBAU, 0xDAU};
    static const uint8_t stop_header[2] = {0xBAU, 0x5AU};
    uint8_t frame[FRAME_SIZE];
    HANDLE port;
    FILE *video;
    long file_size;
    unsigned long frame_count;
    unsigned long frame_index;
    unsigned long loop_count;
    double next_frame_time;
    int response;
    int board_stopped = 0;
    int session_active = 0;
    int fatal_error = 0;
    int interactive_mode = (argc == 1) ? 1 : 0;
    char default_video[MAX_PATH];
    const char *port_name;
    const char *video_path;
    int wait_status;

    if (interactive_mode != 0) {
        port_name = "COM8";
        if (!find_default_video(default_video)) {
            fprintf(stderr,
                    "Cannot automatically find BadApple_full_128x64_12fps.bin.\n");
            pause_before_close(interactive_mode);
            return 1;
        }
        video_path = default_video;
        printf("Double-click mode: using %s.\n", port_name);
        printf("Frame file: %s\n", video_path);
    } else if (argc == 3) {
        port_name = argv[1];
        video_path = argv[2];
    } else {
        fprintf(stderr, "Usage: %s COM7 BadApple_full_128x64_12fps.bin\n", argv[0]);
        return 1;
    }

    video = fopen(video_path, "rb");
    if (video == NULL) {
        fprintf(stderr, "Cannot open frame file: %s\n", video_path);
        pause_before_close(interactive_mode);
        return 1;
    }
    fseek(video, 0L, SEEK_END);
    file_size = ftell(video);
    rewind(video);
    if (file_size <= 0L || (file_size % (long)FRAME_SIZE) != 0L) {
        fprintf(stderr, "Invalid frame file size: %ld bytes.\n", file_size);
        fclose(video);
        pause_before_close(interactive_mode);
        return 1;
    }
    frame_count = (unsigned long)(file_size / (long)FRAME_SIZE);

    port = open_port(port_name);
    if (port == INVALID_HANDLE_VALUE) {
        fclose(video);
        pause_before_close(interactive_mode);
        return 1;
    }

    SetConsoleCtrlHandler(console_handler, TRUE);
    while (InterlockedCompareExchange(&stop_requested, 0L, 0L) == 0L) {
        session_active = 0;
        printf("Waiting on %s: quickly press USER three times to play.\n",
               port_name);
        wait_status = wait_for_board(port);
        if (wait_status == WAIT_OLD_FIRMWARE) {
            fprintf(stderr,
                    "Old Bad Apple firmware detected. Flash the latest stm32.elf.\n");
            fatal_error = 1;
            break;
        }
        if (wait_status != WAIT_READY) {
            if (InterlockedCompareExchange(&stop_requested, 0L, 0L) == 0L) {
                fatal_error = 1;
            }
            break;
        }

        /* Give the board time to enter its frame-header receive loop. */
        Sleep(20U);

        printf("Playing %lu frames at %u FPS.\n", frame_count, FRAME_RATE);
        printf("Quickly press USER three times again to stop this playback.\n");
        printf("Press Ctrl+C to close the computer sender.\n");
        rewind(video);
        loop_count = 0U;
        board_stopped = 0;
        session_active = 1;
        next_frame_time = (double)GetTickCount64();

        while (InterlockedCompareExchange(&stop_requested, 0L, 0L) == 0L &&
               board_stopped == 0 && fatal_error == 0) {
            for (frame_index = 0U; frame_index < frame_count; ++frame_index) {
                double now;

                if (InterlockedCompareExchange(&stop_requested, 0L, 0L) != 0L) {
                    break;
                }
                if (fread(frame, 1U, FRAME_SIZE, video) != FRAME_SIZE ||
                    !write_all(port, frame_header, sizeof(frame_header))) {
                    fatal_error = 1;
                    break;
                }

                response = read_frame_response(port);
                if (response == RESPONSE_BOARD_STOP) {
                    board_stopped = 1;
                    break;
                }
                if (response != RESPONSE_FRAME_READY) {
                    fprintf(stderr, "Frame %lu DMA-ready response failed.\n",
                            frame_index + 1U);
                    fatal_error = 1;
                    break;
                }

                if (!write_all(port, frame, FRAME_SIZE)) {
                    fatal_error = 1;
                    break;
                }

                response = read_frame_response(port);
                if (response == RESPONSE_BOARD_STOP) {
                    board_stopped = 1;
                    break;
                }
                if (response != RESPONSE_ACK) {
                    fprintf(stderr, "Frame %lu display ACK failed.\n",
                            frame_index + 1U);
                    fatal_error = 1;
                    break;
                }

                next_frame_time += 1000.0 / (double)FRAME_RATE;
                now = (double)GetTickCount64();
                if (next_frame_time > now) {
                    Sleep((DWORD)(next_frame_time - now));
                }
            }

            if (InterlockedCompareExchange(&stop_requested, 0L, 0L) == 0L &&
                board_stopped == 0 && fatal_error == 0) {
                ++loop_count;
                printf("Complete video loop %lu finished.\n", loop_count);
                rewind(video);
            }
        }

        if (fatal_error != 0) {
            break;
        }
        if (board_stopped != 0) {
            session_active = 0;
            printf("Playback stopped by USER triple click.\n");
            PurgeComm(port, PURGE_RXCLEAR | PURGE_TXCLEAR);
        }
    }

    if (fatal_error == 0 && session_active != 0) {
        printf("Stopping playback and closing sender...\n");
        if (!write_all(port, stop_header, sizeof(stop_header)) ||
            !read_one(port, ACK_BYTE, "STOP ACK")) {
            fatal_error = 1;
        }
    }

    printf("Sender closed.\n");
    CloseHandle(port);
    fclose(video);
    pause_before_close(interactive_mode);
    return (fatal_error == 0) ? 0 : 1;
}
