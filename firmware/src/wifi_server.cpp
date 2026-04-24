#include "wifi_server.h"
#include "wifi_config.h"

#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"

#include <stdio.h>
#include <string.h>

// ---- HTML dashboard ------------------------------------------------
// Served at GET /  — JavaScript polls GET /s every 500 ms for JSON stats.
// Kept compact so it fits comfortably in a single TCP segment.
static const char INDEX_HTML[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Connection: close\r\n\r\n"
    "<!DOCTYPE html><html><head><meta charset=utf-8>"
    "<title>LOREM</title>"
    "<style>"
    "body{margin:2em;font-family:monospace;background:#111;color:#0f0}"
    "h2{border-bottom:1px solid #0f0;padding-bottom:.3em}"
    "td{padding:4px 14px;border:1px solid #030}"
    "</style></head><body>"
    "<h2>LOREM Robot</h2>"
    "<div id=d>connecting...</div>"
    "<script>"
    "(function tick(){"
    "fetch('/s').then(r=>r.json()).then(d=>{"
    "var t=d.t.map((v,i)=>i+':'+v).join(' | ');"
    "document.getElementById('d').innerHTML="
    "'<table>'"
    "+'<tr><td>Battery</td><td>'+d.b.toFixed(2)+' V</td></tr>'"
    "+'<tr><td>ToF (mm)</td><td>'+t+'</td></tr>'"
    "+'<tr><td>Encoder L</td><td>'+d.el+'</td></tr>'"
    "+'<tr><td>Encoder R</td><td>'+d.er+'</td></tr>'"
    "+'</table>';"
    "}).finally(()=>setTimeout(tick,500));"
    "})();"
    "</script></body></html>";

// ---- shared state --------------------------------------------------
static RobotStats* g_stats;

// ---- HTTP ----------------------------------------------------------
static void send_stats_json(struct tcp_pcb* pcb) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Connection: close\r\n\r\n"
        "{\"b\":%.2f"
        ",\"t\":[%d,%d,%d,%d,%d,%d]"
        ",\"el\":%ld,\"er\":%ld}",
        (double)g_stats->battery,
        (int)g_stats->tof[0], (int)g_stats->tof[1], (int)g_stats->tof[2],
        (int)g_stats->tof[3], (int)g_stats->tof[4], (int)g_stats->tof[5],
        (long)g_stats->enc_l, (long)g_stats->enc_r);
    tcp_write(pcb, buf, (u16_t)n, TCP_WRITE_FLAG_COPY);
}

static void http_err_cb(void* /*arg*/, err_t /*err*/) {
    // PCB is already freed by lwIP when err callback fires — nothing to do.
}

static err_t http_recv_cb(void* /*arg*/, struct tcp_pcb* pcb,
                           struct pbuf* p, err_t /*err*/) {
    if (!p) {
        tcp_close(pcb);
        return ERR_OK;
    }

    // Single-pbuf GET is enough for a browser request
    bool is_stats = (p->len >= 7 && memcmp(p->payload, "GET /s ", 7) == 0);
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);

    if (is_stats) {
        send_stats_json(pcb);
    } else {
        // INDEX_HTML is in flash (.rodata) — safe to pass without copy
        tcp_write(pcb, INDEX_HTML, sizeof(INDEX_HTML) - 1, 0);
    }
    tcp_output(pcb);
    tcp_close(pcb);
    return ERR_OK;
}

static err_t http_accept_cb(void* /*arg*/, struct tcp_pcb* client, err_t err) {
    if (err != ERR_OK || !client) return ERR_VAL;
    tcp_err(client, http_err_cb);
    tcp_recv(client, http_recv_cb);
    return ERR_OK;
}

// ---- core 1 entry --------------------------------------------------
static void wifi_server_thread() {
    if (cyw43_arch_init()) return;

    cyw43_arch_enable_sta_mode();

    // Attempt connection; retry indefinitely so the server comes up
    // automatically after boot even if the AP is slow to appear.
    while (cyw43_arch_wifi_connect_timeout_ms(
               WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000) != 0) {
        sleep_ms(2000);
    }

    // HTTP server
    struct tcp_pcb* srv = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (srv) {
        tcp_bind(srv, IP_ANY_TYPE, WIFI_HTTP_PORT);
        srv = tcp_listen(srv);
        tcp_accept(srv, http_accept_cb);
    }

    // UDP broadcast PCB
    struct udp_pcb* udp = udp_new();
    ip4_addr_t bcast;
    IP4_ADDR(&bcast, 255, 255, 255, 255);

    absolute_time_t next_udp = make_timeout_time_ms(500);

    while (true) {
        cyw43_arch_poll();

        if (time_reached(next_udp)) {
            char buf[128];
            int n = snprintf(buf, sizeof(buf),
                "{\"b\":%.2f"
                ",\"t\":[%d,%d,%d,%d,%d,%d]"
                ",\"el\":%ld,\"er\":%ld}",
                (double)g_stats->battery,
                (int)g_stats->tof[0], (int)g_stats->tof[1], (int)g_stats->tof[2],
                (int)g_stats->tof[3], (int)g_stats->tof[4], (int)g_stats->tof[5],
                (long)g_stats->enc_l, (long)g_stats->enc_r);

            if (udp) {
                struct pbuf* pb = pbuf_alloc(PBUF_TRANSPORT, (u16_t)n, PBUF_RAM);
                if (pb) {
                    memcpy(pb->payload, buf, n);
                    udp_sendto(udp, pb, &bcast, WIFI_UDP_PORT);
                    pbuf_free(pb);
                }
            }
            next_udp = make_timeout_time_ms(500);
        }

        sleep_ms(1);
    }
}

void wifi_server_launch(RobotStats* stats) {
    g_stats = stats;
    multicore_launch_core1(wifi_server_thread);
}
