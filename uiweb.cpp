#include "uiweb.hpp"
#include <thread>
#include <iostream>


//UiParser UiWeb::uiParser;
const std::string UiWeb::TAG = "[uiweb] ";





void UiWeb::startServerThread()
{
    isRunning = true;
        std::thread t1(&UiWeb::serverLoop,this); // passing 'this' by value
        t1.detach();
    std::cout<<TAG<< "started server thread\n";
}

void UiWeb::sendString(std::string s)
{
    //std::cout<<TAG<<"sending to ui dev: "<<s<<std::endl;

}

void  UiWeb::serverLoop()
{
    std::cout<<TAG<<" receiving loop started "<<std::endl;
    {
        /* Server context handle */
        struct mg_context *ctx;

        /* Initialize the library */
        mg_init_library(0);

        /* Start the server */
        ctx = mg_start(NULL, 0, NULL);

        /* Add some handler */
        mg_set_request_handler(ctx, "/auto", handlerAuto, (void*)0);
      //  mg_set_request_handler(ctx, "/manual", handlerManual, (void*)0);

        mg_set_request_handler(ctx, "/joy.js", joyjshandler, (void*)0);
        mg_set_request_handler(ctx, "/favicon.ico", faviconhandler, (void*)0);

            /* Add HTTP site to open a websocket connection */
                mg_set_request_handler(ctx, "/manual", WebSocketStartHandler, 0);
            /* WS site for the websocket connection */
                mg_set_websocket_handler(ctx,
                                         "/manual",
                                         WebSocketConnectHandler,
                                         WebSocketReadyHandler,
                                         WebsocketDataHandler,
                                         WebSocketCloseHandler,
                                         0);


        while(1){

       usleep(250000);
            InformWebsockets(ctx);

        }
        /* Stop the server */
        mg_stop(ctx);

        /* Un-initialize the library */
        mg_exit_library();
    }

}

 int UiWeb::joyjshandler(struct mg_connection *conn, void *ignored){    mg_send_file(conn, "joy.js");
return 1;}
 int UiWeb::faviconhandler(struct mg_connection *conn, void *ignored){    mg_send_file(conn, "favicon.ico");
return 1;}

 int UiWeb::handlerManual(struct mg_connection *conn, void *ignored)
{
    std::cout<<"main.cpp handlerManual  "<<std::endl;

      mg_send_file(conn, "manual.html");
    return 1;//200; /* HTTP state 200 = OK */
}
int UiWeb::handlerAuto(struct mg_connection *conn, void *ignored)
{
    std::cout<<"main.cpp handlerManual  "<<std::endl;

      mg_send_file(conn, "auto.html");
    return 1;//200; /* HTTP state 200 = OK */
}

int
UiWeb::WebSocketStartHandler(struct mg_connection *conn, void *cbdata)
{
    mg_send_file(conn, "websocket.xhtml");



    return 1;
}



int
UiWeb::WebSocketConnectHandler(const struct mg_connection *conn, void *cbdata)
{
    struct mg_context *ctx = mg_get_context(conn);
    int reject = 1;
    int i;

    mg_lock_context(ctx);
    for (i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_clients[i].conn == NULL) {
            ws_clients[i].conn = (struct mg_connection *)conn;
            ws_clients[i].state = 1;
            mg_set_user_connection_data(ws_clients[i].conn,
                                        (void *)(ws_clients + i));
            reject = 0;
            break;
        }
    }
    mg_unlock_context(ctx);

    fprintf(stdout,
            "Websocket client %s\r\n\r\n",
            (reject ? "rejected" : "accepted"));
    return reject;
}


void
UiWeb::WebSocketReadyHandler(struct mg_connection *conn, void *cbdata)
{
    const char *text = "Hello from the websocket ready handler";
    struct t_ws_client *client = (t_ws_client *)mg_get_user_connection_data(conn);

    mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, text, strlen(text));
    fprintf(stdout, "Greeting message sent to websocket client\r\n\r\n");
   // ASSERT(client->conn == conn);
   // ASSERT(client->state == 1);

    client->state = 2;
}


int
UiWeb::WebsocketDataHandler(struct mg_connection *conn,
                     int bits,
                     char *data,
                     size_t len,
                     void *cbdata)
{
    struct t_ws_client *client = (t_ws_client *)mg_get_user_connection_data(conn);
    //ASSERT(client->conn == conn);
    //ASSERT(client->state >= 1);
    std::cout<<"main.cpp websocket data handler  "<<std::endl;

    fprintf(stdout, "Websocket got %lu bytes of ", (unsigned long)len);
    switch (((unsigned char)bits) & 0x0F) {
    case MG_WEBSOCKET_OPCODE_CONTINUATION:
        fprintf(stdout, "continuation");
        break;
    case MG_WEBSOCKET_OPCODE_TEXT:
        fprintf(stdout, "text");
        break;
    case MG_WEBSOCKET_OPCODE_BINARY:
        fprintf(stdout, "binary");
        break;
    case MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE:
        fprintf(stdout, "close");
        break;
    case MG_WEBSOCKET_OPCODE_PING:
        fprintf(stdout, "ping");
        break;
    case MG_WEBSOCKET_OPCODE_PONG:
        fprintf(stdout, "pong");
        break;
    default:
        fprintf(stdout, "unknown(%1xh)", ((unsigned char)bits) & 0x0F);
        break;
    }
    fprintf(stdout, " data:\r\n");
    fwrite(data, len, 1, stdout);
    fprintf(stdout, "\r\n\r\n");
std::cout<<std::endl;
    return 1;
}


void
UiWeb::WebSocketCloseHandler(const struct mg_connection *conn, void *cbdata)
{
    struct mg_context *ctx = mg_get_context(conn);
    struct t_ws_client *client = (t_ws_client *)mg_get_user_connection_data(conn);
    //ASSERT(client->conn == conn);
    //ASSERT(client->state >= 1);

    mg_lock_context(ctx);
    while (client->state == 3) {
        /* "inform" state, wait a while */
        mg_unlock_context(ctx);

        usleep(1000);
        mg_lock_context(ctx);
    }
    client->state = 0;
    client->conn = NULL;
    mg_unlock_context(ctx);

    fprintf(stdout,
            "Client dropped from the set of webserver connections\r\n\r\n");
}


void
UiWeb::InformWebsockets(struct mg_context *ctx)
{
    static unsigned long cnt = 0;
    char text[32];
    size_t textlen;
    int i;

    sprintf(text, "%lu", ++cnt);
    textlen = strlen(text);

    for (i = 0; i < MAX_WS_CLIENTS; i++) {
        int inform = 0;

        mg_lock_context(ctx);
        if (ws_clients[i].state == 2) {
            /* move to "inform" state */
            ws_clients[i].state = 3;
            inform = 1;
        }
        mg_unlock_context(ctx);

        if (inform) {
            mg_websocket_write(ws_clients[i].conn,
                               MG_WEBSOCKET_OPCODE_TEXT,
                               text,
                               textlen);
            mg_lock_context(ctx);
            ws_clients[i].state = 2;
            mg_unlock_context(ctx);
        }
    }
}


