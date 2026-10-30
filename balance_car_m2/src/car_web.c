#define _POSIX_C_SOURCE 200809L

#include "car_control.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define UART_DEVICE "/dev/ttyS1"
#define HTTP_PORT 8080
#define REQUEST_SIZE 4096

static volatile sig_atomic_t stop_requested;

/* Embedded page: deploying the executable alone is enough. */
static const char page_html[] =
    "<!doctype html>\n"
    "<html lang=\"zh-CN\"><head><meta charset=\"utf-8\">\n"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
    "<title>平衡车控制</title><style>\n"
    ":root{font-family:system-ui,sans-serif;color-scheme:dark;background:#0e1726;color:#edf3ff}\n"
    "*{box-sizing:border-box}body{margin:0;min-height:100dvh;display:grid;place-items:center;padding:20px}\n"
    "main{width:min(100%,440px);padding:24px;border-radius:22px;background:#1a293e;box-shadow:0 12px 40px #0006}\n"
    "h1{text-align:center;font-size:1.55rem;margin:0 0 8px}p{text-align:center;color:#b9c9de;margin:0 0 22px}\n"
    ".grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;align-items:stretch}\n"
    "button{font:inherit;font-weight:700;color:white;background:#345a8a;border:0;border-radius:15px;\n"
    "min-height:72px;padding:12px 6px;cursor:pointer;touch-action:none;user-select:none;-webkit-user-select:none}\n"
    "button:active{filter:brightness(1.25)}.wide{grid-column:1/4}.left{grid-column:1}\n"
    ".presets{grid-column:1/4;display:grid;grid-template-columns:repeat(2,1fr);gap:12px}\n"
    ".steer-title{grid-column:1/4;text-align:center;margin:10px 0 0;font-weight:700}\n"
    ".steer-value{display:grid;place-items:center;font-weight:700;font-variant-numeric:tabular-nums}\n"
    ".stop{background:#d4304d;font-size:1.25rem}.readout{margin-top:22px;padding:16px;border-radius:14px;\n"
    "background:#101d2e;display:grid;gap:8px;font-variant-numeric:tabular-nums}\n"
    "#message{min-height:24px;margin-top:16px;color:#fecf87;text-align:center;font-size:.95rem}\n"
    "</style></head><body><main><h1>平衡车控制</h1><p>速度点按调整，转向按住生效</p>\n"
    "<div class=\"grid\"><button class=\"wide\" id=\"up\">+10 RPM · 加速 / 前进</button>\n"
    "<button class=\"wide\" id=\"down\">−10 RPM · 减速 / 后退</button>\n"
    "<div class=\"presets\"><button id=\"speed50\">50 RPM</button>\n"
    "<button id=\"speedminus50\">-50 RPM</button></div>\n"
    "<div class=\"steer-title\">转向角速度</div>\n"
    "<button id=\"steerdown\" aria-label=\"减小转向角速度\">−</button>\n"
    "<div class=\"steer-value\" id=\"steervalue\">30 °/s</div>\n"
    "<button id=\"steerup\" aria-label=\"增大转向角速度\">+</button>\n"
    "<button class=\"left\" id=\"left\">左转<br>按住</button>\n"
    "<button class=\"stop\" id=\"stop\">STOP</button>\n"
    "<button id=\"right\">右转<br>按住</button></div>\n"
    "<div class=\"readout\"><div>Speed: <strong id=\"speed\">—</strong> RPM</div>\n"
    "<div>Yaw: <strong id=\"yaw\">—</strong> deg/s</div></div>\n"
    "<div id=\"message\" role=\"status\" aria-live=\"polite\"></div></main>\n";

static const char page_script[] =
    "<script>\n"
    "const $=id=>document.getElementById(id);\n"
    "function show(s){$('speed').textContent=s.speed_rpm;$('yaw').textContent=s.yaw_rate_dps}\n"
    "function message(text){$('message').textContent=text}\n"
    "let pending=Promise.resolve(), held=null, steerRate=30;\n"
    "async function request(path,method='POST'){\n"
    "  const response=await fetch(path,{method,cache:'no-store'});\n"
    "  if(!response.ok)throw Error('HTTP '+response.status);\n"
    "  show(await response.json());message('');\n"
    "}\n"
    "function command(path){\n"
    "  pending=pending.catch(()=>{}).then(()=>request(path)).catch(error=>{\n"
    "    message('操作失败：'+error.message+'；请检查小车状态');\n"
    "  });\n"
    "  return pending;\n"
    "}\n"
    "$('up').onclick=()=>command('/api/speed/up');\n"
    "$('down').onclick=()=>command('/api/speed/down');\n"
    "$('speed50').onclick=()=>command('/api/speed/50');\n"
    "$('speedminus50').onclick=()=>command('/api/speed/-50');\n"
    "$('steerup').onclick=()=>{steerRate=Math.min(150,steerRate+10);$('steervalue').textContent=steerRate+' °/s'};\n"
    "$('steerdown').onclick=()=>{steerRate=Math.max(0,steerRate-10);$('steervalue').textContent=steerRate+' °/s'};\n"
    "$('stop').onclick=()=>{held=null;command('/api/stop')};\n"
    "function release(event){\n"
    "  if(held!==null&&event.pointerId===held){held=null;command('/api/turn/stop')}\n"
    "}\n"
    "for(const id of ['left','right']){\n"
    "  const button=$(id);\n"
    "  button.addEventListener('pointerdown',event=>{\n"
    "    if(held!==null)return;event.preventDefault();held=event.pointerId;\n"
    "    button.setPointerCapture(event.pointerId);command('/api/turn/'+id+'/'+steerRate);\n"
    "  });\n"
    "  button.addEventListener('pointerup',release);\n"
    "  button.addEventListener('pointercancel',release);\n"
    "  button.addEventListener('lostpointercapture',release);\n"
    "}\n"
    "function leave(){\n"
    "  if(held!==null){held=null;command('/api/turn/stop')}\n"
    "}\n"
    "document.addEventListener('visibilitychange',()=>{if(document.hidden)leave()});\n"
    "window.addEventListener('pagehide',()=>{\n"
    "  if(held!==null)held=null;\n"
    "  navigator.sendBeacon('/api/turn/stop','');\n"
    "});\n"
    "pending=request('/api/status','GET').catch(error=>message('无法获取状态：'+error.message));\n"
    "</script></body></html>\n";

static void handle_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

/* One response per connection. Suppress SIGPIPE if the browser disconnects. */
static int send_all(int fd, const char *data, size_t length)
{
    size_t sent = 0;

    while (sent < length) {
        ssize_t count = send(fd, data + sent, length - sent, MSG_NOSIGNAL);

        if (count > 0) {
            sent += (size_t)count;
        } else if (count < 0 && errno == EINTR && !stop_requested) {
            continue;
        } else {
            if (count == 0) {
                errno = EPIPE;
            }
            return -1;
        }
    }
    return 0;
}

static int respond_parts(int fd, int code, const char *reason,
                         const char *type, const char *body,
                         const char *body_tail)
{
    char headers[320];
    size_t body_length = strlen(body);
    size_t tail_length = strlen(body_tail);
    size_t length = body_length + tail_length;
    int size = snprintf(headers, sizeof(headers),
                        "HTTP/1.1 %d %s\r\n"
                        "Content-Type: %s\r\n"
                        "Content-Length: %zu\r\n"
                        "Cache-Control: no-store\r\n"
                        "X-Content-Type-Options: nosniff\r\n"
                        "Connection: close\r\n\r\n",
                        code, reason, type, length);

    if (size < 0 || (size_t)size >= sizeof(headers)) {
        errno = EOVERFLOW;
        return -1;
    }
    if (send_all(fd, headers, (size_t)size) < 0) {
        return -1;
    }
    if (send_all(fd, body, body_length) < 0) {
        return -1;
    }
    return send_all(fd, body_tail, tail_length);
}

static int respond(int fd, int code, const char *reason,
                   const char *type, const char *body)
{
    return respond_parts(fd, code, reason, type, body, "");
}

static int respond_status(int fd, CarControl *car)
{
    char json[96];
    int size = snprintf(json, sizeof(json),
                        "{\"speed_rpm\":%d,\"yaw_rate_dps\":%d}\n",
                        car->target_speed_rpm, car->target_yaw_rate_dps);

    if (size < 0 || (size_t)size >= sizeof(json)) {
        errno = EOVERFLOW;
        return -1;
    }
    return respond(fd, 200, "OK", "application/json; charset=utf-8", json);
}

/* Only accept the Web presets (0, 10, ..., 150), never an unchecked UART value. */
static int parse_turn_rate(const char *text, int *dps)
{
    char *end;
    long value;

    if (*text < '0' || *text > '9' ||
        (*text == '0' && text[1] != '\0')) {
        return -1;
    }
    errno = 0;
    value = strtol(text, &end, 10);
    if (errno != 0 || *end != '\0' || value < 0 || value > 150 ||
        value % 10 != 0) {
        return -1;
    }
    *dps = (int)value;
    return 0;
}

static int dispatch(int fd, const char *method, const char *path,
                    CarControl *car)
{
    int result;
    int turn_rate;

    if (strcmp(method, "GET") == 0 && strcmp(path, "/") == 0) {
        return respond_parts(fd, 200, "OK", "text/html; charset=utf-8",
                             page_html, page_script);
    }
    if (strcmp(method, "GET") == 0 && strcmp(path, "/api/status") == 0) {
        return respond_status(fd, car);
    }
    if (strcmp(method, "POST") != 0) {
        return respond(fd, 404, "Not Found", "text/plain; charset=utf-8",
                       "Not Found\n");
    }
    if (strcmp(path, "/api/speed/up") == 0) {
        result = CarControl_SpeedUp(car);
    } else if (strcmp(path, "/api/speed/down") == 0) {
        result = CarControl_SpeedDown(car);
    } else if (strcmp(path, "/api/speed/50") == 0) {
        result = CarControl_SetSpeed(car, 50);
    } else if (strcmp(path, "/api/speed/-50") == 0) {
        result = CarControl_SetSpeed(car, -50);
    } else if (strcmp(path, "/api/speed/stop") == 0) {
        result = CarControl_StopSpeed(car);
    } else if (strncmp(path, "/api/turn/left/", sizeof("/api/turn/left/") - 1U) == 0 &&
               parse_turn_rate(path + sizeof("/api/turn/left/") - 1U, &turn_rate) == 0) {
        result = CarControl_SetYawRate(car, turn_rate);
    } else if (strncmp(path, "/api/turn/right/", sizeof("/api/turn/right/") - 1U) == 0 &&
               parse_turn_rate(path + sizeof("/api/turn/right/") - 1U, &turn_rate) == 0) {
        result = CarControl_SetYawRate(car, -turn_rate);
    } else if (strcmp(path, "/api/turn/left") == 0) {
        result = CarControl_TurnLeft(car);
    } else if (strcmp(path, "/api/turn/right") == 0) {
        result = CarControl_TurnRight(car);
    } else if (strcmp(path, "/api/turn/stop") == 0) {
        result = CarControl_StopTurn(car);
    } else if (strcmp(path, "/api/stop") == 0) {
        result = CarControl_Stop(car);
    } else {
        return respond(fd, 404, "Not Found", "text/plain; charset=utf-8",
                       "Not Found\n");
    }
    if (result < 0) {
        perror("CarControl command");
        return respond(fd, 500, "Internal Server Error",
                       "application/json; charset=utf-8",
                       "{\"error\":\"control failed; verify car state\"}\n");
    }
    /* Local targets reflect successful UART writes, not STM32 acknowledgments. */
    return respond_status(fd, car);
}

static int handle_client(int fd, CarControl *car)
{
    char request[REQUEST_SIZE];
    char method[8], path[96], version[16], extra;
    size_t used = 0;
    char *end;
    char *line_end;
    struct timeval timeout = { .tv_sec = 2, .tv_usec = 0 };

    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0 ||
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
        return -1;
    }
    /* Read only bounded headers; close after one request (no keep-alive). */
    for (;;) {
        ssize_t count;

        if (stop_requested) {
            return 0;
        }
        if (used == sizeof(request) - 1U) {
            return respond(fd, 413, "Content Too Large", "text/plain",
                           "Request headers too large\n");
        }
        count = recv(fd, request + used, sizeof(request) - used - 1U, 0);
        if (count < 0 && errno == EINTR && !stop_requested) {
            continue;
        }
        if (count <= 0) {
            return -1;
        }
        used += (size_t)count;
        request[used] = '\0';
        end = strstr(request, "\r\n\r\n");
        if (end != NULL) {
            break;
        }
    }
    /* Reject embedded NULs in headers instead of parsing a truncated request. */
    if (memchr(request, '\0', (size_t)(end - request) + 4U) != NULL) {
        return respond(fd, 400, "Bad Request", "text/plain", "Bad Request\n");
    }
    line_end = strstr(request, "\r\n");
    if (line_end == NULL || line_end > end ||
        (size_t)(line_end - request) >= 160U) {
        return respond(fd, 400, "Bad Request", "text/plain", "Bad Request\n");
    }
    *line_end = '\0';
    if (sscanf(request, "%7s %95s %15s %c", method, path, version, &extra) != 3 ||
        (strcmp(version, "HTTP/1.0") != 0 &&
         strcmp(version, "HTTP/1.1") != 0)) {
        return respond(fd, 400, "Bad Request", "text/plain", "Bad Request\n");
    }
    if (stop_requested) {
        return 0;
    }
    return dispatch(fd, method, path, car);
}

int main(void)
{
    CarControl car;
    struct sigaction action = { 0 };
    struct sockaddr_in address = { 0 };
    int server_fd = -1;
    int reuse = 1;
    int exit_status = EXIT_FAILURE;

    action.sa_handler = handle_signal;
    if (sigemptyset(&action.sa_mask) < 0 ||
        sigaction(SIGINT, &action, NULL) < 0 ||
        sigaction(SIGTERM, &action, NULL) < 0) {
        perror("signal setup");
        return EXIT_FAILURE;
    }
    if (CarControl_Init(&car, UART_DEVICE) < 0) {
        perror("CarControl_Init");
        return EXIT_FAILURE;
    }
    if (stop_requested) {
        goto cleanup;
    }
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0 ||
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse,
                   sizeof(reuse)) < 0) {
        perror("HTTP socket");
        goto cleanup;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(HTTP_PORT);
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0 ||
        listen(server_fd, 8) < 0) {
        perror("HTTP bind/listen");
        goto cleanup;
    }
    printf("car_web listening on 0.0.0.0:%d; Ctrl+C to stop\n", HTTP_PORT);
    fflush(stdout);
    exit_status = EXIT_SUCCESS;

    while (!stop_requested) {
        struct pollfd watched = { .fd = server_fd, .events = POLLIN,
                                  .revents = 0 };
        int ready = poll(&watched, 1, 500);
        int client_fd;

        if (ready < 0 && errno == EINTR) {
            continue;
        }
        if (ready < 0 || (ready > 0 &&
                          (watched.revents & (POLLERR | POLLHUP | POLLNVAL)))) {
            if (ready >= 0) {
                errno = EIO;
            }
            perror("HTTP poll");
            exit_status = EXIT_FAILURE;
            break;
        }
        if (ready == 0 || stop_requested) {
            continue;
        }
        client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0 && errno == EINTR) {
            continue;
        }
        if (client_fd < 0) {
            perror("HTTP accept");
            exit_status = EXIT_FAILURE;
            break;
        }
        if (handle_client(client_fd, &car) < 0 && !stop_requested) {
            perror("HTTP client");
        }
        (void)close(client_fd);
    }

cleanup:
    if (server_fd >= 0) {
        (void)close(server_fd);
    }
    /* Close already attempts speed=0 and yaw=0, even on server startup failure. */
    CarControl_Close(&car);
    return exit_status;
}