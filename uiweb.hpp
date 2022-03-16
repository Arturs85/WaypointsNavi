#ifndef UIWEB_HPP
#define UIWEB_HPP


// Server side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <string>
#include "uiparser.hpp"
#include "civetweb.h"

#define MAX_WS_CLIENTS (5)


class UiWeb{
public:
    static const std::string TAG;

    bool isRunning = true;
    void startServerThread();
    void sendString(std::string s);
    static UiWeb uiWeb;
private:
    void serverLoop();
    static int joyjshandler(mg_connection *conn, void *ignored);
    static int faviconhandler(mg_connection *conn, void *ignored);
    static int handlerManual(mg_connection *conn, void *ignored);
    static int handlerAuto(mg_connection *conn, void *ignored);
    static int WebSocketStartHandler(mg_connection *conn, void *cbdata);
    struct mg_context *ctx;

    struct t_ws_client {
        /* Handle to the connection, used for mg_read/mg_write */
        struct mg_connection *conn;

        /*
            WebSocketConnectHandler sets state to 1 ("connected")
            the connect handler can accept or reject a connection, but it cannot
            send or receive any data at this state

            WebSocketReadyHandler sets state to 2 ("ready")
            reading and writing is possible now

            WebSocketCloseHandler sets state to 0
            the websocket is about to be closed, reading and writing is no longer
            possible this callback can be used to cleanup allocated resources

            InformWebsockets is called cyclic every second, and sends some data
            (a counter value) to all websockets in state 2
        */
        int state;
    };
    static t_ws_client ws_clients[MAX_WS_CLIENTS];

   static int WebSocketConnectHandler(const mg_connection *conn, void *cbdata);
   static void WebSocketReadyHandler(mg_connection *conn, void *cbdata);
   static int WebsocketDataHandler(mg_connection *conn, int bits, char *data, size_t len, void *cbdata);
   static void WebSocketCloseHandler(const mg_connection *conn, void *cbdata);
   static void InformWebsockets(mg_context *ctx, std::string textString);
};

#endif // UIWEB_HPP
