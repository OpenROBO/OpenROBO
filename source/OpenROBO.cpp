#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdarg.h>
#include <assert.h>

#include "SocketCom.h"
#include "OpenROBO.h"

#ifndef OPENROBO_MAKECONNECTION_TIMEOUT_MSEC
#define OPENROBO_MAKECONNECTION_TIMEOUT_MSEC (3*1000)
#endif

#ifndef OPENROBO_READWIRTEMEMORY_DEFAULT_SIZE
#define OPENROBO_READWIRTEMEMORY_DEFAULT_SIZE (128)
#endif

#ifndef OPENROBO_MESSAGE_BUFFER_DEFAULT_SIZE
#define OPENROBO_MESSAGE_BUFFER_DEFAULT_SIZE (1024)
#endif

#ifndef OPENROBO_MESSAGE_BUFFER_SIZE_STEP
#define OPENROBO_MESSAGE_BUFFER_SIZE_STEP (1024)
#endif

#ifdef OPENROBO_NDEBUG

#define DBGPRINTF(...) do{}while(0)
#define DBGFLUSH(...) do{}while(0)
#define DBGABORT(...) do{}while(0)

#else

#define DBGPRINTF printf
#define DBGFLUSH() do{fflush(stdout);}while(0)
#define DBGABORT() do{assert(0);}while(0)

#endif

#ifndef OPENROBO_TRACE_MESSAGE

#define TRACE_PRINTF(...) do{}while(0)
#define TRACE_MESSAGE(msg,size) do{}while(0)
#define TRACE_END() do{}while(0)

#else

#define TRACE_PRINTF(...) fprintf(stderr, __VA_ARGS__)
#define TRACE_MESSAGE(msg, size) do{ size_t i; for (i = 0; i < size; i++) {fputc(msg[i], stderr);} }while(0)
#define TRACE_END() do{ fputc('\n', stderr), fflush(stderr); }while(0)

#endif

#define OPENROBO_MESSAGE_SIZE_STR_SIZE sizeof("00000000")


/**
* The origin of following macro is TinyCThread
* TinyCThread (https://tinycthread.github.io)
* Copyright (c) 2012 Marcus Geelnard
* Copyright (c) 2013-2016 Evan Nemerson
* under the zlib/libpng license
*
* @def _Thread_local
* Thread local storage keyword.
* A variable that is declared with the @c _Thread_local keyword makes the
* value of the variable local to each thread (known as thread-local storage,
* or TLS). Example usage:
* @code
* // This variable is local to each thread.
* _Thread_local int variable;
* @endcode
* @note The @c _Thread_local keyword is a macro that maps to the corresponding
* compiler directive (e.g. @c __declspec(thread)).
* @note This directive is currently not supported on Mac OS X (it will give
* a compiler error), since compile-time TLS is not supported in the Mac OS X
* executable format. Also, some older versions of MinGW (before GCC 4.x) do
* not support this directive, nor does the Tiny C Compiler.
* @hideinitializer
*/
#if !(defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201102L)) && !defined(_Thread_local)
 #if defined(__GNUC__) || defined(__INTEL_COMPILER) || defined(__SUNPRO_CC) || defined(__IBMCPP__)
  #define _Thread_local __thread
 #else
  #define _Thread_local __declspec(thread)
 #endif
#elif defined(__GNUC__) && defined(__GNUC_MINOR__) && (((__GNUC__ << 8) | __GNUC_MINOR__) < ((4 << 8) | 9))
 #define _Thread_local __thread
#endif

struct _OpenROBO_Message_buffer {
  char *p;
  size_t size;
};
static _Thread_local struct _OpenROBO_Message_buffer OpenROBO_Message_commonBuffer = {NULL, 0};

static SocketCom OpenROBO_acceptSocket = SOCKETCOM_INITIALIZER;
static uint16_t OpenROBO_acceptPort = OPENROBO_DEFAULT_ACCEPT_PORT;
static _Thread_local int OpenROBO_isMainThread = 0;
#define OpenROBO_selfSubsystemName OpenROBO_subsystemTable.infos[0].id


const char* const OpenROBO_SubsystemName_TASKPLANNER = "TP";

const char* const OpenROBO_MessageHeader_Start = "Start;";
const char* const OpenROBO_MessageHeader_Stop  = "stop;";
const char* const OpenROBO_MessageHeader_Wait  = "Wait;";
const char* const OpenROBO_MessageHeader_Return   = "Return;";
const char* const OpenROBO_MessageHeader_Read   = "read;";
const char* const OpenROBO_MessageHeader_Write   = "write;";

static _Thread_local char OpenROBO_threadID[OPENROBO_THREAD_ID_SIZE];

static void OpenROBO_Message_setDestinationID(char* message, const char* destinationID);
static void OpenROBO_Message_setSourceID(char* message, const char* sourceID);
static int OpenROBO_Socket_forwardReturnMessage(const char* originalMessage, const char *returnMessage);
static int OpenROBO_Socket_sendMessage(const char* destinationID, const char* message, const char* suffix);
static int OpenROBO_Socket_recvMessage(SocketCom* sock, char **message);

int OpenROBO_ReadWriteMemory_put(const char *key, const char *message);
const char *OpenROBO_ReadWriteMemory_get(const char *key);
int OpenROBO_ReadWriteMemory_init(int size);
int OpenROBO_Message_buffer_realloc(struct _OpenROBO_Message_buffer* buf, size_t size);
static int OpenROBO_Socket_sendReturnMessageBySystem(const char* originalMessage, const char *returnMessage);

/*
    subsystemID format is "SubsystemName" such as "TASKPLANNER"
    threadID format is "FunctionName@SubsystemName" such as "GraspBolt@TASKPLANNER"
*/
static char* OpenROBO_generateThreadID(const char *subsystemID, const char *functionName, char *threadID)
{
  size_t subsystemIDSize, functionNameSize;
  subsystemIDSize = strlen(subsystemID)+1;
  functionNameSize = strlen(functionName)+1;

  if ((subsystemIDSize + functionNameSize) > OPENROBO_THREAD_ID_SIZE) {
    return NULL;
  }

  memcpy(&threadID[0], subsystemID, subsystemIDSize);
  threadID[subsystemIDSize-1] = '@';
  memcpy(&threadID[subsystemIDSize], functionName, functionNameSize+1);

  return threadID;
}

static char* OpenROBO_generateThreadIDFromMessage(const char *message, char *threadID)
{
  const char *functionName;
  const char *subsystemID;
  OpenROBO_Message_GetSubject(message, &functionName);
  OpenROBO_Message_GetDestinationID(message, &subsystemID);
  OpenROBO_generateThreadID(subsystemID, functionName, threadID);
  OpenROBO_free((void*)subsystemID);
  OpenROBO_free((void*)functionName);
  return threadID;
}

/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

   OpenROBO_subsystemTable

   いつでも接続可能なサブシステムの一覧と接続先情報。
   TaskPlannerではAcceptConnection()、その他のsubsystemではMakeConnection()の際に作成される。

   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */

typedef struct {
  char id[OPENROBO_SUBSYSTEM_ID_SIZE];
  char ip[OPENROBO_IP_STR_LEN+1];
  uint16_t port;
} OpenROBO_subsystemTable_info_t;

typedef struct {
  OpenROBO_subsystemTable_info_t infos[OPENROBO_AGENTS_COMMECTION_MAX];
  size_t infosSize;
} OpenROBO_subsystemTable_t;

// TaskPlannerの場合: OpenROBO_subsystemTable[0]はTaskPlanner(自身)の情報, それ以降はその他のエージェントの情報
// TaskPlanner以外の場合: OpenROBO_subsystemTable[0]は自身の情報, OpenROBO_subsystemTable[1]はTaskPlannerの情報, それ以降はその他のエージェントの情報
static _Thread_local OpenROBO_subsystemTable_t OpenROBO_subsystemTable = {
  {{{0,}, SOCKETCOM_INITIALIZER},},
  0
};

static int OpenROBO_hasSubsystemInfo(const char* id)
{
  size_t i;
  for (i = 0; i < OpenROBO_subsystemTable.infosSize; i++) {
    if (strcmp(OpenROBO_subsystemTable.infos[i].id, id) == 0) {
      return 1;
    }
  }
  return 0;
}

static int OpenROBO_hasSubsystemInfos(const char* const ids[])
{
  size_t i;
  for (i = 0; ids[i] != NULL; i++) {
    if (!OpenROBO_hasSubsystemInfo(ids[i])) {
      return 0;
    }
  }
  return 1;
}

/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

   OpenROBO_sockList


   通信に使用するsocketのリスト

   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */

typedef struct _OpenROBO_sockList {
  char id[OPENROBO_THREAD_ID_SIZE];
  SocketCom sock;
  struct _OpenROBO_sockList* next;
} OpenROBO_sockList_t;

static _Thread_local OpenROBO_sockList_t* OpenROBO_sockList = NULL;

static size_t OpenROBO_sockList_getLen()
{
  size_t c = 0;
  OpenROBO_sockList_t *p = OpenROBO_sockList;
  if (p == NULL) {
    return c;
  }

  while (1) {
    c++;
    if (p->next == NULL) {
      break;
    }
    p = p->next;
  }

  return c;
}

static OpenROBO_sockList_t* OpenROBO_sockList_getLast()
{
  OpenROBO_sockList_t *p = OpenROBO_sockList;
  if (p == NULL) {
    return NULL;
  }
  while (1) {
    if (p->next == NULL) {
      break;
    }
    p = p->next;
  }
  return p;
}

static OpenROBO_sockList_t* OpenROBO_sockList_createNew()
{
  _OpenROBO_sockList *p, *n;
  n = (OpenROBO_sockList_t*)OpenROBO_malloc(sizeof(OpenROBO_sockList_t));
  if (n == NULL) {
    return NULL;
  }

  // init
  SocketCom_Init(&n->sock);
  n->next = NULL;

  p = OpenROBO_sockList_getLast();
  if (p == NULL) {
    OpenROBO_sockList = n;
  } else {
    p->next = n;
  }

  return n;
}

static int OpenROBO_sockList_delete(OpenROBO_sockList_t* del)
{
  OpenROBO_sockList_t *p1, *p2;
  p1 = OpenROBO_sockList;
  p2 = NULL;
  while (1) {
    if (p1 == NULL) {
      return OpenROBO_Return_Error;
    }
    if (p1 == del) {
      break;
    }
    p2 = p1;
    p1 = p1->next;
  }

  if (p2 == NULL) {
    OpenROBO_sockList = p1->next;
  } else {
    p2->next = p1->next;
  }

  SocketCom_Dispose(&p1->sock);
  OpenROBO_free(p1);

  return OpenROBO_Return_Success;
}

static void OpenROBO_sockList_deleteAll()
{
  while (OpenROBO_sockList_delete(OpenROBO_sockList) == OpenROBO_Return_Success);
}

static OpenROBO_sockList_t* OpenROBO_sockList_findByID(const char* id)
{
  OpenROBO_sockList_t *p;
  p = OpenROBO_sockList;
  while (1) {
    if (p == NULL) {
      break;
    }
    if (strcmp(p->id, id) == 0) {
      break;
    }
    p = p->next;
  }

  return p;
}

static OpenROBO_sockList_t* OpenROBO_sockList_findBySocketCom(SocketCom *sock)
{
  OpenROBO_sockList_t *p;
  p = OpenROBO_sockList;
  while (1) {
    if (p == NULL) {
      return NULL;
    }
    if (SocketCom_Equal(&p->sock, sock)) {
      break;
    }
    p = p->next;
  }

  return p;
}

static SocketCom* OpenROBO_sockList_findSocketCom(const char *ID)
{
  OpenROBO_sockList_t *s = OpenROBO_sockList_findByID(ID);
  if (s == NULL) {
    return NULL;
  }
  return &s->sock;
}

static void OpenROBO_sockList_deleteBySocketCom(SocketCom *sock)
{
  OpenROBO_sockList_t *s = OpenROBO_sockList_findBySocketCom(sock);
  if (s == NULL) {
    return;
  }

  OpenROBO_sockList_delete(s);
}

static OpenROBO_sockList_t* OpenROBO_sockList_connect(const char* destinationID)
{
  int res;
  size_t i;
  OpenROBO_subsystemTable_t *table = &OpenROBO_subsystemTable;
  for (i = 0; i < table->infosSize; i++) {
    if (strcmp(destinationID, table->infos[i].id) == 0) {
      break;
    }
  }
  if (i >= table->infosSize) {
    return NULL;
  }

  OpenROBO_sockList_t *s = OpenROBO_sockList_createNew();
  if (s == NULL) {
    return NULL;
  }

  res = SocketCom_Create(&s->sock);
  if (res != SOCKETCOM_SUCCESS) {
    return NULL;
  }
  res = SocketCom_ConnectTo(&s->sock, table->infos[i].ip, table->infos[i].port);
  if (res != SOCKETCOM_SUCCESS) {
    return NULL;
  }
  res = SocketCom_Send(&s->sock, OpenROBO_threadID, strlen(OpenROBO_threadID)+1);
  if (res != SOCKETCOM_SUCCESS) {
    return NULL;
  }
  strcpy(s->id, table->infos[i].id);

  return s;
}

//TODO
#if 0
static int OpenROBO_cloneConnection(const OpenROBO_subsystemTable_t* src, OpenROBO_subsystemTable_t *dst, const char* firstMessage)
{
  size_t firstMessageSize = strlen(firstMessage) + 1;
  int res;
  size_t i;
  for (i = 0; i < src->infosSize; i++) {
    OpenROBO_sockList_t *s = OpenROBO_sockList_createNew();
    if (s == NULL) {
      return OpenROBO_Return_Error;
    }
    res = SocketCom_Create(&s->sock);
    if (res != SOCKETCOM_SUCCESS) {
      return OpenROBO_Return_Error;
    }
    res = SocketCom_ConnectTo(&s->sock, src->infos[i].ip, src->infos[i].port);
    if (res != SOCKETCOM_SUCCESS) {
      return OpenROBO_Return_Error;
    }
    res = SocketCom_Send(&s->sock, firstMessage, firstMessageSize);
    if (res != SOCKETCOM_SUCCESS) {
      return OpenROBO_Return_Error;
    }
    strcpy(s->id, src->infos[i].id);
  }

  return OpenROBO_Return_Success;
}
#endif

static int OpenROBO_Socket_createAcceptSocket(uint16_t *_port);

int OpenROBO_Message_buffer_init()
{
  OpenROBO_Message_commonBuffer.size = OPENROBO_MESSAGE_BUFFER_DEFAULT_SIZE;
  OpenROBO_Message_commonBuffer.p = (char *)OpenROBO_malloc(OpenROBO_Message_commonBuffer.size);
  if (OpenROBO_Message_commonBuffer.p == NULL) {
    return OpenROBO_Return_Error;
  }
  OpenROBO_Message_commonBuffer.p[0] = '\0';

  return OpenROBO_Return_Success;
}

void OpenROBO_Message_buffer_term()
{
  if (OpenROBO_Message_commonBuffer.p == NULL) {
    return;
  }
  OpenROBO_free(OpenROBO_Message_commonBuffer.p);
  OpenROBO_Message_commonBuffer.p = NULL;
  OpenROBO_Message_commonBuffer.size = 0;
}

int OpenROBO_StartupMainThread(const char* subsystemName)
{
  int res;
  if (strlen(subsystemName) >= OPENROBO_SUBSYSTEM_ID_SIZE) {
    return OpenROBO_Return_Error;
  }

  OpenROBO_isMainThread = 1;

  res = OpenROBO_ReadWriteMemory_init(OPENROBO_READWIRTEMEMORY_DEFAULT_SIZE);
  if (res != OpenROBO_Return_Success) {
    return res;
  }

  SocketCom_Startup();

  res = OpenROBO_Socket_createAcceptSocket(&OpenROBO_acceptPort);
  if (res != OpenROBO_Return_Success) {
    return res;
  }

  res = OpenROBO_Message_buffer_init();
  if (res != OpenROBO_Return_Success) {
    return res;
  }

  strcpy(OpenROBO_threadID, subsystemName);
  strcpy(OpenROBO_subsystemTable.infos[0].id, subsystemName);
  strcpy(OpenROBO_subsystemTable.infos[0].ip, "127.0.0.1");
  OpenROBO_subsystemTable.infos[0].port = OpenROBO_acceptPort;
  OpenROBO_subsystemTable.infosSize = 1;

  return OpenROBO_Return_Success;
}



/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
   OpenROBO_Thread

   The origin is TinyCThread

   TinyCThread (https://tinycthread.github.io)
   Copyright (c) 2012 Marcus Geelnard
   Copyright (c) 2013-2016 Evan Nemerson
   under the zlib/libpng license

-----------------------------------------------------
     License of TinyCThread
-----------------------------------------------------
Copyright (c) 2012 Marcus Geelnard
Copyright (c) 2013-2016 Evan Nemerson

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not
    claim that you wrote the original software. If you use this software
    in a product, an acknowledgment in the product documentation would be
    appreciated but is not required.

    2. Altered source versions must be plainly marked as such, and must not be
    misrepresented as being the original software.

    3. This notice may not be removed or altered from any source
    distribution.
-----------------------------------------------------
   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */
struct _OpenROBO_Thread_startInfo
{
  int (*func)(int, char *[]);
  OpenROBO_MessageFunction_t msgfunc;
  OpenROBO_subsystemTable_t subsystemTable;
  char *message;
  int argc;
  char** argv;
};

static _Thread_local int OpenROBO_Thread_workingFlag = 1;

static int OpenROBO_Thread_detach(OpenROBO_Thread_t thr)
{
#if defined(_OPENROBO_WIN32_)
  /* https://stackoverflow.com/questions/12744324/how-to-detach-a-thread-on-windows-c#answer-12746081 */
  return CloseHandle(thr) != 0 ? OpenROBO_Return_Success : OpenROBO_Return_Error;
#else
  return pthread_detach(thr) == 0 ? OpenROBO_Return_Success : OpenROBO_Return_Error;
#endif
}

/* Thread wrapper function. */
#if defined(_OPENROBO_WIN32_)
static DWORD WINAPI OpenROBO_Thread_start_wrapper(LPVOID aArg)
#elif defined(_OPENROBO_POSIX_)
static void * OpenROBO_Thread_start_wrapper(void * aArg)
#endif
{
  int ret;
  int (*func)(int, char *[]);
  OpenROBO_MessageFunction_t msgfunc;
  char *message;
  int argc;
  char **argv;

  /* Get thread startup information */
  _OpenROBO_Thread_startInfo *ti = (_OpenROBO_Thread_startInfo *) aArg;
  func = ti->func;
  msgfunc = ti->msgfunc;
  message = ti->message;
  argc = ti->argc;
  argv = ti->argv;

  ret = OpenROBO_Message_buffer_init();

  OpenROBO_generateThreadIDFromMessage(message, OpenROBO_threadID);
  OpenROBO_subsystemTable = ti->subsystemTable;

  /* The thread is responsible for freeing the startup information */
  OpenROBO_free((void *)ti);

  /* Call the actual client thread function */
  if (func != NULL) {
    /* Particular Subthread */
    OpenROBO_free((void *)message);
    func(argc, argv);
  } else {
    /* Message Subthread */
    char returnMessage[1024] = "";
    const char *subject;
    OpenROBO_Message_GetSubject(message, &subject);
    OpenROBO_Message_MakeReturnMessage(returnMessage, subject);
    OpenROBO_Message_SetReturnValue(returnMessage, ret);
    OpenROBO_Socket_sendReturnMessageBySystem(message, returnMessage);
    OpenROBO_free((void *)subject);
    if (ret == OpenROBO_Return_Success) { // TODO
      msgfunc(message);
    }
    OpenROBO_free((void *)message);
  }

  OpenROBO_Message_buffer_term();

  OpenROBO_CheckWorking(); // for clear socket buffer

  OpenROBO_sockList_deleteAll();

#if defined(_OPENROBO_WIN32_)
  return 0;
#else
  return NULL;
#endif
}

static int OpenROBO_Message_clone(const char* originalMessage, char** clonedMessage)
{
  char *p;
  size_t size;
  size = strlen(originalMessage) + 1;
  p = (char *)OpenROBO_malloc(size);
  if (p == NULL) {
    DBGABORT();
    return OpenROBO_Return_Error;
  }
  strcpy(p, originalMessage);
  *clonedMessage = p;

  return OpenROBO_Return_Success;
}

static int OpenROBO_Thread_create_common(int (*func)(int, char *[]), OpenROBO_MessageFunction_t msgfunc, char* message, int argc, char *argv[])
{
  OpenROBO_Thread_t thr;
  _OpenROBO_Thread_startInfo* ti = (_OpenROBO_Thread_startInfo*)OpenROBO_malloc(sizeof(_OpenROBO_Thread_startInfo));
  if (ti == NULL) {
    return OpenROBO_Return_Error;
  }

  if (OpenROBO_Message_clone(message, &ti->message) != OpenROBO_Return_Success) {
    return OpenROBO_Return_Error;
  }
  ti->func = func;
  ti->msgfunc = msgfunc;
  ti->argc = argc;
  ti->argv = argv;
  ti->subsystemTable = OpenROBO_subsystemTable;

  /* Create the thread */
#if defined(_OPENROBO_WIN32_)
  thr = CreateThread(NULL, 0, OpenROBO_Thread_start_wrapper, (LPVOID) ti, 0, NULL);
#elif defined(_OPENROBO_POSIX_)
  if(pthread_create(&thr, NULL, OpenROBO_Thread_start_wrapper, (void *)ti) != 0)
  {
    thr = 0;
  }
#endif

  /* Did we fail to create the thread? */
  if(!thr)
  {
    OpenROBO_free(ti);
    return OpenROBO_Return_Error;
  }

  OpenROBO_Thread_detach(thr);

  return OpenROBO_Return_Success;
}

int OpenROBO_Thread_CreateSubthread(int (*func)(int, char *[]), const char* funcName, int argc, char *argv[])
{
  char threadID[OPENROBO_THREAD_ID_SIZE];
  OpenROBO_generateThreadID(OpenROBO_threadID, funcName, threadID);
  if (OpenROBO_sockList_findByID(threadID) != NULL) {
    return OpenROBO_Return_DoubleCreateSubthread;
  }

  char *message = (char *)malloc(sizeof(char)*1024);
  if (message == NULL) {
    return OpenROBO_Return_Error;
  }

  OpenROBO_Message_MakeOperationMessage(message, funcName);
  OpenROBO_Message_setSourceID(message, OpenROBO_threadID);
  OpenROBO_Message_setDestinationID(message, OpenROBO_threadID);

  return OpenROBO_Thread_create_common(func, NULL, message, argc, argv);
}

int OpenROBO_Thread_CreateOperationThread(OpenROBO_MessageFunction_t func, char* message)
{
  int res;
  char threadID[OPENROBO_THREAD_ID_SIZE];

  OpenROBO_generateThreadIDFromMessage(message, threadID);
  if (OpenROBO_sockList_findByID(threadID) != NULL) {
    res = OpenROBO_Return_DoubleCreateSubthread;
  } else {
    res = OpenROBO_Thread_create_common(NULL, func, message, 0, NULL);
  }

  if (res != OpenROBO_Return_Success) {
    char returnMessage[1024] = "";
    const char *subject;
    OpenROBO_Message_GetSubject(message, &subject);
    OpenROBO_Message_MakeReturnMessage(returnMessage, subject);
    OpenROBO_Message_SetReturnValue(returnMessage, res);
    OpenROBO_Socket_sendReturnMessageBySystem(message, returnMessage);
    OpenROBO_free((void*)subject);
  }

  return res;
}

typedef struct _OpenROBO_joinThreadQueue {
  const char* message;
  struct _OpenROBO_joinThreadQueue *next;
} OpenROBO_joinThreadQueue_t;

static OpenROBO_joinThreadQueue_t *OpenROBO_JoinThread_returnMessageList = NULL;

static OpenROBO_joinThreadQueue_t *OpenROBO_JoinThread_waitList = NULL;

static const char* OpenROBO_joinThreadQueue_findByFunctionName(OpenROBO_joinThreadQueue_t** list, const char* functionName)
{
  OpenROBO_joinThreadQueue_t *q;
  const char *f;

  q = *list;

  while (q != NULL) {
    int res;
    OpenROBO_Message_GetSubject(q->message, &f);
    res = strcmp(f, functionName);
    OpenROBO_free((void*)f);
    if (res == 0) {
      return q->message;
    }
    q = q->next;
  }

  return NULL;
}

static int OpenROBO_joinThreadQueue_append(OpenROBO_joinThreadQueue_t** list, const char* message)
{
  OpenROBO_joinThreadQueue_t *p, *l;

  p = (OpenROBO_joinThreadQueue_t *)OpenROBO_malloc(sizeof(OpenROBO_joinThreadQueue_t));
  if (p == NULL) {
    return OpenROBO_Return_Error;
  }
  p->message = message;
  p->next = NULL;

  l = *list;
  if (l == NULL) {
    *list = p;
  } else {
    while (l->next != NULL) {
      l = l->next;
    }
    l->next = p;
  }
  return OpenROBO_Return_Success;
}

static int OpenROBO_joinThreadQueue_delete(OpenROBO_joinThreadQueue_t** list, const char* message)
{
  OpenROBO_joinThreadQueue_t *q1, *q2;

  q1 = *list;
  q2 = NULL;
  while (1) {
    if (q1->message == message) {
      break;
    }
    q2 = q1;
    q1 = q1->next;
  }
  if (q2 == NULL) {
    *list = q1->next;
  } else {
    q2->next = q1->next;
  }
  OpenROBO_free((void*)q1->message);
  OpenROBO_free(q1);

  return OpenROBO_Return_Success;
}

static int OpenROBO_JoinThread_storeThreadReturnMessage(const char* message)
{
  if (!OpenROBO_isMainThread) {
    return OpenROBO_Return_Error;
  }
  const char *functionName;
  const char *exitMessage;
  int res;

  OpenROBO_Message_GetSubject(message, &functionName);
  exitMessage = OpenROBO_joinThreadQueue_findByFunctionName(&OpenROBO_JoinThread_waitList, functionName);
  OpenROBO_free((void *)functionName);
  if (exitMessage == NULL) {
    char *_message;
    res = OpenROBO_Message_clone(message, &_message);
    if (res != OpenROBO_Return_Success) {
      return res;
    }
    return OpenROBO_joinThreadQueue_append(&OpenROBO_JoinThread_returnMessageList, _message);
  } else {
    res = OpenROBO_Socket_forwardReturnMessage(exitMessage, message);
    if (res != OpenROBO_Return_Success) {
      return res;
    }
    OpenROBO_joinThreadQueue_delete(&OpenROBO_JoinThread_waitList, exitMessage);
    return OpenROBO_Return_Success;
  }
}

int OpenROBO_JoinThread_JoinQueue(const char* message)
{
  if (!OpenROBO_isMainThread) {
    return OpenROBO_Return_Error;
  }


  const char *functionName;
  const char *returnMessage;

  OpenROBO_Message_GetSubject(message, &functionName);
  returnMessage = OpenROBO_joinThreadQueue_findByFunctionName(&OpenROBO_JoinThread_returnMessageList, functionName);
  OpenROBO_free((void *)functionName);
  if (returnMessage == NULL) {
    int res;
    char *_message;
    res = OpenROBO_Message_clone(message, &_message);
    if (res != OpenROBO_Return_Success) {
      return res;
    }
    return OpenROBO_joinThreadQueue_append(&OpenROBO_JoinThread_waitList, _message);
  } else {
    int res;
    res = OpenROBO_Socket_forwardReturnMessage(message, returnMessage);
    if (res != OpenROBO_Return_Success) {
      return res;
    }
    OpenROBO_joinThreadQueue_delete(&OpenROBO_JoinThread_returnMessageList, returnMessage);
    return OpenROBO_Return_Success;
  }
}

int OpenROBO_WaitForStopMessage(void)
{
  int res;
  char buf[1];

  if (OpenROBO_isMainThread) {
    return 1;
  }

  if (!OpenROBO_Thread_workingFlag) {
    return OpenROBO_Return_Success;
  }

  res = SocketCom_Recv(&OpenROBO_sockList->sock, buf, sizeof(buf), NULL);
  if (res != SOCKETCOM_SUCCESS) { //fatal error
    DBGABORT();
    OpenROBO_Thread_workingFlag = 0;
    return OpenROBO_Return_Error;
  }

  if (buf[0] != '\0') { // unknown condition
    DBGABORT();
    OpenROBO_Thread_workingFlag = 0;
    return OpenROBO_Return_Error;
  }

  OpenROBO_Thread_workingFlag = 0;

  return OpenROBO_Return_Success;
}

int OpenROBO_CheckWorking(void)
{
  int res;
  char buf[1];

  if (OpenROBO_isMainThread) {
    return 1;
  }

  while (1) {
    if (!SocketCom_IsRecvable(&OpenROBO_sockList->sock)) {
      return OpenROBO_Thread_workingFlag;
    }

    res = SocketCom_RecvEx(&OpenROBO_sockList->sock, buf, sizeof(buf), NULL, MSG_PEEK);
    if (res != SOCKETCOM_SUCCESS) { //fatal error
      DBGABORT();
      OpenROBO_Thread_workingFlag = 0;
      return OpenROBO_Thread_workingFlag;
    }

    if (buf[0] != '\0') {
      break;
    }

    OpenROBO_Thread_workingFlag = 0;
    res = SocketCom_Recv(&OpenROBO_sockList->sock, buf, sizeof(buf), NULL);
    if (res != SOCKETCOM_SUCCESS) {
      DBGABORT();
      return OpenROBO_Thread_workingFlag;
    }
  }

  return OpenROBO_Thread_workingFlag;
}

int OpenROBO_sendExitThreadSignal(const char* message)
{
  char destionationID[OPENROBO_THREAD_ID_SIZE];
  OpenROBO_generateThreadIDFromMessage(message, destionationID);

  OpenROBO_sockList_t *s;
  int res;
  char signal[1] = {'\0'};
  s = OpenROBO_sockList_findByID(destionationID);
  if (s == NULL) {
    return OpenROBO_Return_NonConnection;
  }

  res = SocketCom_Send(&s->sock, signal, sizeof(signal));
  if (res != SOCKETCOM_SUCCESS) {
    return OpenROBO_Return_Error;
  }

  return OpenROBO_Return_Success;
}

/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

   OpenROBO_Socket

   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */

static int OpenROBO_Socket_recvMessage(SocketCom* sock, char **message)
{
  int res;
  size_t size;
  char *str;
  char sizeStr[OPENROBO_MESSAGE_SIZE_STR_SIZE];

  res = SocketCom_RecvAll(sock, sizeStr, sizeof(sizeStr));
  if (res != SOCKETCOM_SUCCESS) { //error
    if (res == SOCKETCOM_ERROR_DISCONNECTED) {
      return OpenROBO_Return_Disconnected;
    }
    return OpenROBO_Return_Error;
  }
  if (sizeStr[0] == '\0') {
    OpenROBO_Thread_workingFlag = 0;
    memmove(&sizeStr[0], &sizeStr[1], sizeof(sizeStr)-1);
    res = SocketCom_RecvAll(sock, &sizeStr[sizeof(sizeStr)-1], 1);
    if (res != SOCKETCOM_SUCCESS) { //error
      if (res == SOCKETCOM_ERROR_DISCONNECTED) {
        return OpenROBO_Return_Disconnected;
      }
      return OpenROBO_Return_Error;
    }
  }
  sscanf(sizeStr, "%lx", &size);

  str = OpenROBO_Message_commonBuffer.p;
  if (size > OpenROBO_Message_commonBuffer.size) {
    OpenROBO_Message_commonBuffer.p[0] = '\0';
    res = OpenROBO_Message_buffer_realloc(&OpenROBO_Message_commonBuffer, size);
    if (res != OpenROBO_Return_Success) {
      return res;
    }
    OpenROBO_Message_commonBuffer.p = str;
  }

  res = SocketCom_RecvAll(sock, str, size);
  if (res != SOCKETCOM_SUCCESS) { //error
    OpenROBO_free(str);
    if (res == SOCKETCOM_ERROR_DISCONNECTED) {
      return OpenROBO_Return_Disconnected;
    }
    return OpenROBO_Return_Error;
  }

  if (str[size-1] != '\0') { //check //TODO is need?
    OpenROBO_free(str);
    return OpenROBO_Return_Error;
  }

  TRACE_PRINTF("<<< [ <%s> has received \"", OpenROBO_isMainThread ? "MainThread" : OpenROBO_threadID);
  TRACE_MESSAGE(str, size);
  TRACE_PRINTF("\" ] <<<");
  TRACE_END();

  *message = str;
  return OpenROBO_Return_Success;
}

static int OpenROBO_Socket_sendMessage(const char* destinationID, const char* message, const char* suffix)
{
  size_t totalSize;
  size_t messageSize;
  size_t suffixSize = 0;
  char sizeStr[OPENROBO_MESSAGE_SIZE_STR_SIZE];
  char endOfMessage[1]= {'\0'};
  int res;

  SocketCom *sock = OpenROBO_sockList_findSocketCom(destinationID);
  if (sock == NULL) { //not connected
    if (OpenROBO_isMainThread) {
      return OpenROBO_Return_Error;
    }
    OpenROBO_sockList_t* sockList;
    sockList = OpenROBO_sockList_connect(destinationID);
    if (sockList == NULL) {
      return OpenROBO_Return_Error;
    }
    sock = &sockList->sock;
  }

  messageSize = strlen(message);
  if (suffix != NULL) {
    suffixSize = strlen(suffix);
  }
  totalSize = messageSize + suffixSize + 1;


  TRACE_PRINTF(">>> [ <%s> will send \"", OpenROBO_isMainThread ? "MainThread" : OpenROBO_threadID);
  TRACE_MESSAGE(message, messageSize);
  if (suffix != NULL) {
    TRACE_MESSAGE(suffix, suffixSize);
  }
  TRACE_PRINTF("\" ] >>>");
  TRACE_END();

  sprintf(sizeStr, "%lx", totalSize);
  res = SocketCom_Send(sock, sizeStr, sizeof(sizeStr));
  if (res != SOCKETCOM_SUCCESS) {
    return OpenROBO_Return_Error;
  }

  res = SocketCom_Send(sock, message, messageSize);
  if (res != SOCKETCOM_SUCCESS) {
    return OpenROBO_Return_Error;
  }
  if (suffix != NULL) {
    res = SocketCom_Send(sock, suffix, suffixSize);
    if (res != SOCKETCOM_SUCCESS) {
      return OpenROBO_Return_Error;
    }
  }
  res = SocketCom_Send(sock, endOfMessage, sizeof(endOfMessage));
  if (res != SOCKETCOM_SUCCESS) {
    return OpenROBO_Return_Error;
  }

  return OpenROBO_Return_Success;
}

int OpenROBO_Socket_SendCommandMessage(const char* destinationID, char* message)
{
  if (OpenROBO_isMainThread) {
    return OpenROBO_Return_Error;
  }

  OpenROBO_Message_setSourceID(message, OpenROBO_threadID);
  OpenROBO_Message_setDestinationID(message, destinationID);

  return OpenROBO_Socket_sendMessage(destinationID, message, NULL);
}

static int OpenROBO_Socket_forwardReturnMessage(const char* originalMessage, const char *returnMessage)
{
  int res;
  if (!OpenROBO_isMainThread) {
    DBGABORT();
    return OpenROBO_Return_Error;
  }

  const char *originalSourceID;
  OpenROBO_Message_GetSourceID(originalMessage, &originalSourceID);
  res = OpenROBO_Socket_sendMessage(originalSourceID, returnMessage, NULL);
  OpenROBO_free((void*)originalSourceID);
  return res;
}

static int OpenROBO_Socket_sendReturnMessageBySystem(const char* originalMessage, const char *returnMessage)
{
  char additionalMessage[1024] = "";
  const char *functionName;
  const char *originalSourceID;
  OpenROBO_Message_GetSourceID(originalMessage, &originalSourceID);
  OpenROBO_Message_setSourceID(additionalMessage, OpenROBO_threadID);
  OpenROBO_Message_setDestinationID(additionalMessage, originalSourceID);
  OpenROBO_Message_GetSubject(originalMessage, &functionName);
  OpenROBO_Message_SetSubject(additionalMessage, functionName);
  OpenROBO_free((void*)functionName);

  if (OpenROBO_isMainThread) {
    int res;
    res = OpenROBO_Socket_sendMessage(originalSourceID, returnMessage, additionalMessage);
    OpenROBO_free((void*)originalSourceID);
    return res;
  } else {
    OpenROBO_free((void*)originalSourceID);
    return OpenROBO_Socket_sendMessage(OpenROBO_selfSubsystemName, returnMessage, additionalMessage);
  }
}

int OpenROBO_Socket_SendReturnMessage(const char *returnMessage)
{
  if (OpenROBO_isMainThread) {
    DBGABORT();
    return OpenROBO_Return_Error;
  }
  char additionalMessage[1024] = "";

  OpenROBO_Message_setSourceID(additionalMessage, OpenROBO_threadID);

  return OpenROBO_Socket_sendMessage(OpenROBO_selfSubsystemName, returnMessage, additionalMessage);
}

int OpenROBO_Socket_ReceiveReturnMessage(const char* sourceID, char** message)
{
  if (OpenROBO_isMainThread) {
    return OpenROBO_Return_Error;
  }

  SocketCom *sock;
  sock = OpenROBO_sockList_findSocketCom(sourceID);
  if (sock == NULL) { //not connected
    return OpenROBO_Return_Error;
  }

  char *_message;
  int res = OpenROBO_Socket_recvMessage(sock, &_message);
  if (message == NULL) { //TODO
    OpenROBO_free(_message);
  } else {
    *message = _message;
  }

  return res;
}

static int OpenROBO_Socket_recvString(SocketCom* sock, char *str, size_t strSize)
{
  int res;
  unsigned int totalSize = 0;
  while (1) {
    res = SocketCom_Recv(sock, &str[totalSize], 1, NULL);
    if (res != SOCKETCOM_SUCCESS) { //error
      if (res == SOCKETCOM_ERROR_DISCONNECTED) {
        return OpenROBO_Return_Disconnected;
      }
      return OpenROBO_Return_Error;
    }
    totalSize++;

    if (str[totalSize-1] == '\0') {
      break;
    }
    if (totalSize >= strSize) { // size over
      return OpenROBO_Return_BufferOver;
    }
  }

  return OpenROBO_Return_Success;
}

static int OpenROBO_Socket_recvConnectionInfos(SocketCom* sock)
{
  int res;
  char buf[OPENROBO_SUBSYSTEM_ID_SIZE+OPENROBO_IP_STR_LEN+OPENROBO_PORT_STR_LEN+3];

  while (1) {
    res = OpenROBO_Socket_recvString(sock, buf, sizeof(buf));
    if (res != OpenROBO_Return_Success) {
      return res;
    }
    if (buf[0] == '\0') {
      break;
    }

    uint32_t port;
    char *ip_str, *port_str, *agentName;
    ip_str = buf;
    port_str = strchr(buf, ':');
    if (port_str == NULL) {
      return OpenROBO_Return_Error;
    }
    *port_str = '\0';
    port_str++;
    agentName = strchr(port_str, ' ');
    if (agentName == NULL) {
      return OpenROBO_Return_Error;
    }
    *agentName = '\0';
    agentName++;

    if (strlen(ip_str) > OPENROBO_IP_STR_LEN) {
      return OpenROBO_Return_Error;
    }
    // TODO error check ip format
    port = atoi(port_str);
    if (!(port > 0 && port <= 65535)) {
      return OpenROBO_Return_Error;
    }
    if (strlen(agentName) >= OPENROBO_SUBSYSTEM_ID_SIZE) {
      return OpenROBO_Return_Error;
    }

    if (strcmp(agentName, OpenROBO_selfSubsystemName) == 0) {
      continue;
    }

    size_t n = OpenROBO_subsystemTable.infosSize;
    strcpy(OpenROBO_subsystemTable.infos[n].ip, ip_str);
    OpenROBO_subsystemTable.infos[n].port = port;
    strcpy(OpenROBO_subsystemTable.infos[n].id, agentName);
    OpenROBO_subsystemTable.infosSize++;
  }

  return OpenROBO_Return_Success;
}

static int OpenROBO_Socket_sendConnectionInfos(SocketCom* sock)
{
  int res;
  size_t i;
  char buf[OPENROBO_SUBSYSTEM_ID_SIZE+OPENROBO_IP_STR_LEN+OPENROBO_PORT_STR_LEN+3];

  for (i = 0; i < OpenROBO_subsystemTable.infosSize; i++) {
    char *name = OpenROBO_subsystemTable.infos[i].id;
    char *ip = OpenROBO_subsystemTable.infos[i].ip;
    uint16_t port = OpenROBO_subsystemTable.infos[i].port;

    sprintf(buf, "%s:%d %s", ip, port, name);
    res = SocketCom_Send(sock, buf, strlen(buf)+1);
    if (res != SOCKETCOM_SUCCESS) { //error
      return OpenROBO_Return_Error;
    }
  }

  char endMessage[] = {'\0'};
  res = SocketCom_Send(sock, endMessage, sizeof(endMessage));
  if (res != SOCKETCOM_SUCCESS) { //error
    return OpenROBO_Return_Error;
  }

  return OpenROBO_Return_Success;
}

static int OpenROBO_Socket_recvSelfInfo(SocketCom* sock, OpenROBO_subsystemTable_info_t* info)
{
  uint16_t port;
  char *port_str, *agentName;
  char buf[OPENROBO_SUBSYSTEM_ID_SIZE+OPENROBO_PORT_STR_LEN+2];
  int res = OpenROBO_Socket_recvString(sock, buf, sizeof(buf));
  if (res != OpenROBO_Return_Success) {
    return res;
  }

  port_str = buf;
  agentName = strchr(buf, ' ');
  if (agentName == NULL) {
    return OpenROBO_Return_Error;
  }
  *agentName = '\0';
  agentName++;

  port = atoi(port_str);
  if (!(port > 0 && port <= 65535)) {
    return OpenROBO_Return_Error;
  }
  if (strlen(agentName) >= OPENROBO_SUBSYSTEM_ID_SIZE) {
    return OpenROBO_Return_Error;
  }

  SocketCom_GetIpStr(sock, info->ip);
  info->port = port;
  strcpy(info->id, agentName);

  return OpenROBO_Return_Success;
}

static int OpenROBO_Socket_sendSelfInfo(SocketCom* sock)
{
  int res;
  char buf[OPENROBO_SUBSYSTEM_ID_SIZE+OPENROBO_PORT_STR_LEN+2];

  sprintf(buf, "%d %s", OpenROBO_acceptPort, OpenROBO_selfSubsystemName);
  res = SocketCom_Send(sock, buf, strlen(buf)+1);
  if (res != SOCKETCOM_SUCCESS) {
    return OpenROBO_Return_Error;
  }

  return OpenROBO_Return_Success;
}

static int OpenROBO_Socket_acceptNewThread(SocketCom* acceptSock)
{
  int res;
  OpenROBO_sockList_t *s = OpenROBO_sockList_createNew();
  if (s == NULL) {
    return OpenROBO_Return_Error;
  }

  res = SocketCom_Accept(acceptSock, &s->sock);
  if (res != SOCKETCOM_SUCCESS) {
    OpenROBO_sockList_delete(s);
    return OpenROBO_Return_Error;
  }

  res = OpenROBO_Socket_recvString(&s->sock, s->id, sizeof(s->id));
  if (res != OpenROBO_Return_Success) {
    SocketCom_Dispose(&s->sock);
    OpenROBO_sockList_delete(s);
    //return OpenROBO_Return_Error; //TODO
  }

  return OpenROBO_Return_Success;
}

int OpenROBO_Socket_ReceiveMessage(char** message)
{
  int res;
  if (!OpenROBO_isMainThread) {
    DBGABORT();
    return OpenROBO_Return_Error;
  }

  static SocketCom **socks = NULL;
  static int socksLen = 0;

  while (1) {
    int newSocksLen = OpenROBO_sockList_getLen() + 1;
    if (newSocksLen > socksLen) {
      if (socks != NULL) {
        OpenROBO_free(socks);
      }
      socksLen = newSocksLen;
      socks = (SocketCom **)OpenROBO_malloc(sizeof(SocketCom *)*socksLen);
      if (socks == NULL) {
        return OpenROBO_Return_Error;
      }
    }

    int i = 0;
    OpenROBO_sockList_t *p = OpenROBO_sockList;
    while (p != NULL) {
      socks[i] = &p->sock;
      i++;
      p = p->next;
    }
    socks[i] = &OpenROBO_acceptSocket;

    SocketCom_WaitForRecvables(socks, &socksLen);
    for (i = 0; i < socksLen; i++) {
      if (SocketCom_Equal(socks[i], &OpenROBO_acceptSocket)) {
        res = OpenROBO_Socket_acceptNewThread(&OpenROBO_acceptSocket);
        if (res != OpenROBO_Return_Success) {
          return res;
        }
        continue;
      }

      res = OpenROBO_Socket_recvMessage(socks[i], message);
      if (res == OpenROBO_Return_Disconnected) {
        OpenROBO_sockList_t *s = OpenROBO_sockList_findBySocketCom(socks[i]);
        if (OpenROBO_hasSubsystemInfo(s->id)) {
          strcpy(OpenROBO_Message_commonBuffer.p, s->id);
          *message = OpenROBO_Message_commonBuffer.p;
          OpenROBO_sockList_deleteBySocketCom(socks[i]);
          return res;
        }
        OpenROBO_sockList_deleteBySocketCom(socks[i]);
        continue;
      }

      return res;
    }
  }

}

OpenROBO_MessageFunctionEntry_t* OpenROBO_MessageFunctionEntry_Find(OpenROBO_MessageFunctionEntry_t *entry, const char *functionName)
{
  while (entry->func != NULL) {
    if (strcmp(functionName, entry->name) == 0) {
      return entry;
    }
    entry++;
  }

  return NULL;
}

int OpenROBO_ReturnForReadMessage(const char *message)
{
  int res;
  int ret;
  const char *subject;
  const char *memoryMessage;
  const char *originalSourceID;
  char *returnMessage;

  OpenROBO_Message_GetSourceID(message, &originalSourceID);

  OpenROBO_Message_GetSubject(message, &subject);
  memoryMessage = OpenROBO_ReadWriteMemory_get(subject);
  if (memoryMessage == NULL) {
    ret = OpenROBO_Return_NotUpdated;
  } else {
    ret = OpenROBO_Return_Success;
  }
  OpenROBO_Message_GetBuffer(&returnMessage);
  OpenROBO_Message_MakeReturnMessage(returnMessage, subject);
  OpenROBO_Message_SetReturnValue(returnMessage, ret);

  res = OpenROBO_Socket_sendMessage(originalSourceID, returnMessage, memoryMessage);
  if (res != OpenROBO_Return_Success) {
    ret = res;
  }

  OpenROBO_free((void *)originalSourceID);
  OpenROBO_free((void *)subject);

  return res;
}

int OpenROBO_StoreWriteMessage(const char *message)
{
  int res;
  const char *subject;
  char returnMessage[1024] = "";
  OpenROBO_Message_GetSubject(message, &subject);
  res = OpenROBO_ReadWriteMemory_put(subject, message);
  res = res < 0 ? OpenROBO_Return_Error : OpenROBO_Return_Success;

  OpenROBO_Message_MakeReturnMessage(returnMessage, subject);
  OpenROBO_Message_SetReturnValue(returnMessage, res);
  OpenROBO_Socket_sendReturnMessageBySystem(message, returnMessage);

  OpenROBO_free((void*)subject);

  return res;
}

int OpenROBO_Main(OpenROBO_MessageFunctionEntry_t operationEntry[])
{
  char *message;
  while (1) {
    int res;
    const char *functionName;
    res = OpenROBO_Socket_ReceiveMessage(&message);
    if (res != OpenROBO_Return_Success) {
      if (res == OpenROBO_Return_Disconnected) {
        DBGPRINTF("Disconnected <%s>\n", message);
      }
      return res;
    }

    switch (OpenROBO_Message_GetMessageType(message)) {
      case OpenROBO_MessageType_Start:
      {
        OpenROBO_MessageFunctionEntry_t *entry;
        OpenROBO_Message_GetSubject(message, &functionName);
        entry = OpenROBO_MessageFunctionEntry_Find(operationEntry, functionName);
        if (entry == NULL) {
          DBGPRINTF("error: not found \"%s\" at start thread / message:[%s]\n", functionName, message);
          OpenROBO_free((void*)functionName);
          res = OpenROBO_Return_Error;
        } else {
          OpenROBO_free((void*)functionName);
          res = OpenROBO_Thread_CreateOperationThread(entry->func, message);
        }
        if (res == OpenROBO_Return_Success) {
          res = OpenROBO_JoinThread_JoinQueue(message);
        }
        break;
      }
      case OpenROBO_MessageType_Return:
      {
        res = OpenROBO_JoinThread_storeThreadReturnMessage(message);
        break;
      }
      case OpenROBO_MessageType_Wait:
      {
        res = OpenROBO_JoinThread_JoinQueue(message);
        break;
      }
      case OpenROBO_MessageType_Stop:
      {
        res = OpenROBO_sendExitThreadSignal(message);
        break;
      }
      case OpenROBO_MessageType_Read:
      {
        res = OpenROBO_ReturnForReadMessage(message);
        break;
      }
      case OpenROBO_MessageType_Write:
      {
        res = OpenROBO_StoreWriteMessage(message);
        break;
      }
    }
    if (res == OpenROBO_Return_Error) {
      DBGABORT();
      return res;
    }
  }

  return OpenROBO_Return_Success;
}

static int OpenROBO_Socket_createAcceptSocket(uint16_t *_port)
{
  uint16_t port = OPENROBO_DEFAULT_ACCEPT_PORT;
  int res;
  res = SocketCom_Create(&OpenROBO_acceptSocket);
  if (res != SOCKETCOM_SUCCESS) {
    return OpenROBO_Return_Error;
  }
  res = SocketCom_SetReuseaddr(&OpenROBO_acceptSocket);
  if (res != SOCKETCOM_SUCCESS) {
    return res;
  }
  while (1) {
    res = SocketCom_Listen(&OpenROBO_acceptSocket, port);
    if (res == SOCKETCOM_SUCCESS) {
      break;
    }
    if (res != SOCKETCOM_ERROR_BIND) {
      return OpenROBO_Return_Error;
    }
    port++;
    if (port == OPENROBO_DEFAULT_ACCEPT_PORT) {
      return OpenROBO_Return_Error;
    }
    if (port == 0) {
      port++;
    }
  }
  *_port = port;

  return OpenROBO_Return_Success;
}

int OpenROBO_Socket_MakeConnection(const char *ip, uint16_t port)
{
  int res;
  OpenROBO_sockList_t *s = OpenROBO_sockList_createNew();
  if (s == NULL) {
    DBGABORT();
    return OpenROBO_Return_Error;
  }

  SocketCom *sock = &s->sock;
  strcpy(s->id, OpenROBO_SubsystemName_TASKPLANNER);

  if (!OpenROBO_isMainThread) {
    return OpenROBO_Return_Error;
  }


#ifndef  OPENROBO_NDEBUG
  const char progressIndicator[] = {'-', '\\', '|', '/'};
  int progressCount = 0;
#endif
  SocketCom_SetAddr(sock, ip, port);
  while (1) {
    res = SocketCom_Create(sock);
    if (res != SOCKETCOM_SUCCESS) {
      return OpenROBO_Return_Error;
    }

    res = SocketCom_ConnectWithTimeout(sock, OPENROBO_MAKECONNECTION_TIMEOUT_MSEC);
    if (res == SOCKETCOM_SUCCESS) {
      break;
    }

    SocketCom_Dispose(sock);

#ifndef OPENROBO_NDEBUG
    DBGPRINTF("retry to connect [%c]\r", progressIndicator[progressCount]);
    DBGFLUSH();
    progressCount++;
    if (progressCount >= (sizeof(progressIndicator)/sizeof(progressIndicator[0]))) {
      progressCount = 0;
    }
#endif
  }

  DBGPRINTF("Connected to (%s:%d)\n", ip, port);

  res = OpenROBO_Socket_sendSelfInfo(sock);
  if (res != OpenROBO_Return_Success) {
    OpenROBO_sockList_deleteBySocketCom(sock);
    return res;
  }

  res = OpenROBO_Socket_recvConnectionInfos(sock);
  if (res != OpenROBO_Return_Success) {
    OpenROBO_sockList_deleteBySocketCom(sock);
    return res;
  }

  size_t n;
  for (n = 0; n < OpenROBO_subsystemTable.infosSize; n++) {
    OpenROBO_subsystemTable_info_t *info;
    info = &OpenROBO_subsystemTable.infos[n];
    if (strcmp(info->id, OpenROBO_SubsystemName_TASKPLANNER) == 0) {
      strcpy(info->ip, ip);
      break;
    }
  }
  if (n >= OpenROBO_subsystemTable.infosSize) {
    return OpenROBO_Return_Error;
  }

  return OpenROBO_Return_Success;
}

int OpenROBO_Socket_AcceptConnection(uint16_t port, const char* const ids[])
{
  int res;

  if (!OpenROBO_isMainThread) {
    return OpenROBO_Return_Error;
  }

  SocketCom acceptSock = SOCKETCOM_INITIALIZER;
  res = SocketCom_Create(&acceptSock);
  if (res != SOCKETCOM_SUCCESS) {
    return OpenROBO_Return_Error;
  }
  res = SocketCom_SetReuseaddr(&acceptSock);
  if (res != SOCKETCOM_SUCCESS) {
    return res;
  }
  res = SocketCom_Listen(&acceptSock, port);
  if (res != SOCKETCOM_SUCCESS) {
    SocketCom_Dispose(&acceptSock);
    return OpenROBO_Return_Error;
  }

  while (!OpenROBO_hasSubsystemInfos(ids)) {
    size_t n = OpenROBO_subsystemTable.infosSize;
    OpenROBO_subsystemTable_info_t *info = &OpenROBO_subsystemTable.infos[n];
    SocketCom *sock;
    OpenROBO_sockList_t *s = OpenROBO_sockList_createNew();
    if (s == NULL) {
      DBGABORT();
      return OpenROBO_Return_Error;
    }
    sock = &s->sock;

    res = SocketCom_Accept(&acceptSock, sock);
    if (res != SOCKETCOM_SUCCESS) {
      SocketCom_Dispose(&acceptSock);
      return OpenROBO_Return_Error;
    }

    res = OpenROBO_Socket_recvSelfInfo(sock, info);
    if (res != OpenROBO_Return_Success) {
      if (res == OpenROBO_Return_Disconnected) {
        continue; //retry to accept
      }
      DBGABORT();
      SocketCom_Dispose(&acceptSock);
      return OpenROBO_Return_Error;
    }
    if (OpenROBO_hasSubsystemInfo(info->id)) {
      DBGPRINTF("Error: Double Connection <%s>@%s\n", info->id, info->ip);
      DBGABORT();
      return OpenROBO_Return_Error;
    }
    strcpy(s->id, info->id);
    DBGPRINTF("Got info: <%s>(%s:%d)\n", info->id, info->ip, info->port);

    OpenROBO_subsystemTable.infosSize++;
  }

  DBGPRINTF("Completed to Get ALL Connection Information\n");

  for (OpenROBO_sockList_t *s = OpenROBO_sockList; s != NULL; s = s->next) {
    SocketCom *sock = &s->sock;

    res = OpenROBO_Socket_sendConnectionInfos(sock);
    if (res != OpenROBO_Return_Success) {
      OpenROBO_sockList_deleteAll();
      DBGABORT();
      return OpenROBO_Return_Error;
    }
  }

  SocketCom_Dispose(&acceptSock);

  return OpenROBO_Return_Success;
}

/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

   OpenROBO_Message

   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */

static const char * const OpenROBO_Message_paramName_sourceID = "#src";
static const char * const OpenROBO_Message_paramName_destinationID = "#dst";
static const char * const OpenROBO_Message_paramName_subject = "#subject";
static const char * const OpenROBO_Message_paramName_return = "#return";
static const char * const OpenROBO_Message_paramName_time = "#time";

int OpenROBO_CreateMessage(char* message, const char* funcName, const char* destionationID);

void OpenROBO_Message_GetParam(const char *MessageParameter,const char *vname,...)
{
  char *cp,tmp[1000]={0},type[1],*len,*len2;
  const char *ssp = NULL;
  double *dp;
  int *ip,n,i,ci;
  va_list val;
  len = (char*)OpenROBO_malloc(strlen(MessageParameter));
  len2 = (char*)OpenROBO_malloc(strlen(MessageParameter));
  sprintf(tmp,";%s=",vname);//-- ;変数名=　というフォーマットに合わせて検索
  ssp = strstr(MessageParameter,tmp);//-- 検索結果のポインタをセット

  if(ssp == NULL){
    DBGPRINTF("GetPram(): not found \"%s\" in [%s]\n", vname, MessageParameter);
    assert(ssp != NULL);
    //		getchar();
    return;
  }

  sscanf(ssp,"%[^=]=(%c%d),%[^;]",tmp,&type[0],&n,len);//-- 検索結果を分解して変数型(type)と数(n)、値(len)を読み取る
  va_start(val,vname);//-- 可変引数のポインタをaにセット
  switch(type[0]){
    case 'i':
      //-- もし、変数のタイプがint型ならint型のポインタに引数ポインタをセット
      ip = va_arg(val,int*);
      for(i = 0;i < n;i++){
        sscanf(len,"%d,%s",(ip + i),len2);
        strcpy(len,len2);
      }
      break;
    case 'd':
      dp = va_arg(val,double*);
      for(i = 0;i < n;i++){
        sscanf(len,"%lf,%s",(dp + i),len2);
        strcpy(len,len2);
      }
      break;
    case 'c':
      cp = va_arg(val,char*);
      ci = va_arg(val,int);
      for(i = 0;i < n;i++){
        sscanf(len,"%[^,],%s",(cp + i*ci),len2);
        strcpy(len,len2);
      }
      break;
    case 's':
      cp = va_arg(val,char*);
      sscanf(len,"%[^,],%s",cp,len2);
      break;
    case 'b': {
                unsigned char* ret;
                int tmp;
                ret = va_arg(val, unsigned char*);
                for (i = 0; i < n; i++) {
                  sscanf(len, "%x,%s", &tmp, len2);
                  ret[i] = tmp;
                  strcpy(len, len2);
                }
                break;
              }
  }
  va_end(val);
  OpenROBO_free((void*)len);
  OpenROBO_free((void*)len2);
}

int OpenROBO_Message_HasParam(const char* message, const char* name)
{
  const char* p = message;
  size_t name_len = strlen(name);

  while (1) {
    p = strstr(p, name);
    if (p == NULL) {
      return 0;
    }
    p += name_len;
    if (p[0] == '=') {
      break;
    }
  }

  return 1;
}

void OpenROBO_Message_GetParam_string(const char *message, const char *name, const char **str)
{
  const char* end;
  const char* p = message;
  size_t name_len = strlen(name);

  while (1) {
    p = strstr(p, name);
    if (p == NULL) {
      DBGPRINTF("error: not found param [%s]\n", name);
      DBGPRINTF("       message [%s]\n", message);
      DBGPRINTF("       %s\n", OpenROBO_isMainThread ? "MainThread" : "OtherThread");
      DBGABORT();
      *str = NULL;
      return;
    }
    p += name_len;
    if (p[0] == '=') {
      break;
    }
  }
  p += 1;

  if (strncmp(p, "(s1),", 5) != 0) {
    assert(strncmp(p, "(s1),", 5) == 0);
    *str = NULL;
    return;
  }
  p += 5;

  end = strchr(p, ';');
  if (end == NULL) {
    end = strchr(p, '\0');
  }

  char *_str;
  _str = (char *)OpenROBO_malloc(end - p + 1);
  if (_str == NULL) {
    *str = NULL;
    assert(_str != NULL);
    return;
  }

  strncpy(_str, p, end - p);
  _str[end-p] = '\0';

  *str = _str;
}

void OpenROBO_Message_GetSourceID(const char* message, const char** sourceID)
{
  OpenROBO_Message_GetParam_string(message, OpenROBO_Message_paramName_sourceID, sourceID);
}

void OpenROBO_Message_GetDestinationID(const char* message, const char** destinationID)
{
  OpenROBO_Message_GetParam_string(message, OpenROBO_Message_paramName_destinationID, destinationID);
}

void OpenROBO_Message_GetSubject(const char* message, const char** subject)
{
  OpenROBO_Message_GetParam_string(message, OpenROBO_Message_paramName_subject, subject);
}

void OpenROBO_Message_GetParam_TMatrix(const char *message, const char *name, double TMatrix[4][4])
{
  OpenROBO_Message_GetParam(message, name, TMatrix);
}

void OpenROBO_Message_GetParam_double(const char *message, const char *name, double *value)
{
  OpenROBO_Message_GetParam(message, name, value);
}

void OpenROBO_Message_GetParam_doubleArray(const char *message,const char *name, double *values, unsigned int n)
{
  OpenROBO_Message_GetParam(message, name, values);
}

void OpenROBO_Message_GetParam_int(const char *message, const char *name, int *value)
{
  OpenROBO_Message_GetParam(message, name, value);
}

void OpenROBO_Message_GetParam_intArray(const char *message,const char *name, int *values, unsigned int n)
{
  OpenROBO_Message_GetParam(message, name, values);
}

void OpenROBO_Message_GetParam_byteArray(const char *message,const char *name, unsigned char *values, unsigned int n)
{
  OpenROBO_Message_GetParam(message, name, values);
}

void OpenROBO_Message_GetReturnValue(const char *message, int *value)
{
  OpenROBO_Message_GetParam_int(message, OpenROBO_Message_paramName_return, value);
}

void OpenROBO_Message_GetTime(const char* message, double* time)
{
  OpenROBO_Message_GetParam_double(message, OpenROBO_Message_paramName_time, time);
}

/**
 * パラメータからメッセージを生成し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *
 * @param[out] MessageParameter 生成したメッセージが追記されるメッセージ
 * @param[in] type メッセージの種類(intなら"i", doubleなら"d"など)
 * @param[in] size パラメータの値のサイズ(例:intならsizeof(int)を引数として渡す)
 */
static void OpenROBO_Message_SetParam(char *MessageParameter,const char *type,const int size,const char *vname,...)
{
  const char *cp;
  char tmp[1000] = "";
  const int *ip;
  int i,ci,n;
  const double *dp;
  va_list val;
  sprintf(tmp,";%s=(%c",vname,type[0]);
  strcat(MessageParameter,tmp);
  va_start(val,vname);//可変引数のポインタをaにセット
  switch(type[0]){
    case 'i':
      //もし、変数のタイプがint型ならint型のポインタに引数ポインタをセット
      ip = va_arg(val,const int*);
      n = size/sizeof(int);
      sprintf(tmp,"%d)",n);
      strcat(MessageParameter,tmp);
      for(i = 0;i < n;i++){
        sprintf(tmp,",%d",*(ip + i));
        strcat(MessageParameter,tmp);
      }
      break;
    case 'd':
      dp = va_arg(val,const double*);
      n = size/sizeof(double);
      sprintf(tmp,"%d)",n);
      strcat(MessageParameter,tmp);
      for(i = 0;i < n;i++){
        sprintf(tmp,",%lf",*(dp + i));
        strcat(MessageParameter,tmp);
      }
      break;
    case 'c':
      cp = va_arg(val,const char*);
      ci = va_arg(val,int);
      n = size/ci;
      sprintf(tmp,"%d)",n);
      strcat(MessageParameter,tmp);
      for(i = 0;i < n;i++){
        sprintf(tmp,",%s",(cp + ci*i));
        strcat(MessageParameter,tmp);
      }
      break;
    case 's':
      cp = va_arg(val,const char*);
      n = 1;
      sprintf(tmp,"%d)",n);
      strcat(MessageParameter,tmp);
      sprintf(tmp,",%s",cp);
      strcat(MessageParameter,tmp);
      break;
    case 'b': {
                const unsigned char* p;
                p = va_arg(val, const unsigned char*);
                n = size/sizeof(unsigned char);
                sprintf(tmp, "%d)", n);
                strcat(MessageParameter, tmp);
                for(i = 0;i < n;i++){
                  sprintf(tmp, ",%02x", p[i]);
                  strcat(MessageParameter, tmp);
                }
                break;
              }
  }
  va_end(val);
}

void OpenROBO_Message_SetParam_string(char *message, const char *name, const char *str)
{
  OpenROBO_Message_SetParam(message,"s",sizeof(char),name,str);
}

void OpenROBO_Message_SetSubject(char* message, const char* subject)
{
  OpenROBO_Message_SetParam_string(message, OpenROBO_Message_paramName_subject, subject);
}

void OpenROBO_Message_setTime(char* message, double time)
{
  OpenROBO_Message_SetParam_double(message, OpenROBO_Message_paramName_time, &time);
}

static void OpenROBO_Message_setSourceID(char* message, const char* sourceID)
{
  OpenROBO_Message_SetParam_string(message, OpenROBO_Message_paramName_sourceID, sourceID);
}

static void OpenROBO_Message_setDestinationID(char* message, const char* destinationID)
{
  OpenROBO_Message_SetParam_string(message, OpenROBO_Message_paramName_destinationID, destinationID);
}

void OpenROBO_Message_SetParam_TMatrix(char *message, const char *name, const double TMatrix[4][4])
{
  OpenROBO_Message_SetParam(message,"d",sizeof(double[4][4]),name,TMatrix);
}

void OpenROBO_Message_SetParam_double(char *message, const char *name, const double *value)
{
  OpenROBO_Message_SetParam(message,"d",sizeof(double),name,value);
}

void OpenROBO_Message_SetParam_doubleArray(char *message,const char *name,const double *values, unsigned int n)
{
  OpenROBO_Message_SetParam(message,"d",sizeof(double)*n,name,values);
}

void OpenROBO_Message_SetParam_int(char *message, const char *name, const int *value)
{
  OpenROBO_Message_SetParam(message,"i",sizeof(int),name,value);
}

void OpenROBO_Message_SetParam_intArray(char *message,const char *name,const int *values, unsigned int n)
{
  OpenROBO_Message_SetParam(message,"i",sizeof(int)*n,name,values);
}

void OpenROBO_Message_SetParam_byteArray(char *message,const char *name,const unsigned char *values, unsigned int n)
{
  OpenROBO_Message_SetParam(message,"b",sizeof(unsigned char)*n,name,values);
}

void OpenROBO_Message_SetReturnValue(char *message, int value)
{
  OpenROBO_Message_SetParam_int(message, OpenROBO_Message_paramName_return, &value);
}

int OpenROBO_Message_GetBuffer(char** message)
{
  char *_message = OpenROBO_Message_commonBuffer.p;
  _message[0] = '\0';
  *message = _message;

  return OpenROBO_Return_Success;
}

int OpenROBO_Message_buffer_realloc(struct _OpenROBO_Message_buffer* buf, size_t size)
{
  char *new_buf;
  new_buf = (char *)OpenROBO_malloc(size);
  if (new_buf == NULL) {
    DBGABORT();
    return OpenROBO_Return_Error;
  }
  DBGPRINTF("%s: %ld -> %ld \n", __func__, buf->size, size);
  strcpy(new_buf, buf->p);
  OpenROBO_free(buf->p);
  buf->p = new_buf;
  buf->size = size;

  return OpenROBO_Return_Success;
}

static void OpenROBO_Message_makeMessage(char *message, const char* header, const char* subject)
{
  strcpy(message, header);
  OpenROBO_Message_SetSubject(message, subject);
}

void OpenROBO_Message_MakeOperationMessage(char *message, const char* subject)
{
  OpenROBO_Message_makeMessage(message, OpenROBO_MessageHeader_Start, subject);
}

void OpenROBO_Message_MakeWaitMessage(char *message, const char* subject)
{
  OpenROBO_Message_makeMessage(message, OpenROBO_MessageHeader_Wait, subject);
}

void OpenROBO_Message_MakeStopMessage(char *message, const char* subject)
{
  OpenROBO_Message_makeMessage(message, OpenROBO_MessageHeader_Stop, subject);
}

void OpenROBO_Message_MakeReturnMessage(char *message, const char* subject)
{
  OpenROBO_Message_makeMessage(message, OpenROBO_MessageHeader_Return, subject);
}

void OpenROBO_Message_MakeReadMessage(char *message, const char* subject)
{
  OpenROBO_Message_makeMessage(message, OpenROBO_MessageHeader_Read, subject);
}

void OpenROBO_Message_MakeWriteMessage(char *message, const char* subject)
{
  OpenROBO_Message_makeMessage(message, OpenROBO_MessageHeader_Write, subject);
}

int OpenROBO_Message_GetMessageType(const char *message)
{
    if (message[0] == OpenROBO_MessageHeader_Start[0]) {
      return OpenROBO_MessageType_Start;
    } else if (message[0] == OpenROBO_MessageHeader_Return[0]) {
      return OpenROBO_MessageType_Return;
    } else if (message[0] == OpenROBO_MessageHeader_Wait[0]) {
      return OpenROBO_MessageType_Wait;
    } else if (message[0] == OpenROBO_MessageHeader_Stop[0]) {
      return OpenROBO_MessageType_Stop;
    } else if (message[0] == OpenROBO_MessageHeader_Read[0]) {
      return OpenROBO_MessageType_Read;
    } else if (message[0] == OpenROBO_MessageHeader_Write[0]) {
      return OpenROBO_MessageType_Write;
    }

    return -1;
}

/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
   OpenROBO_ReadWrite

   Thanks for Hatada's Home Page (http://home.a00.itscom.net/hatada/c-tips/hash01.html)
   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */

typedef struct {
  char key[OPENROBO_SUBSYSTEM_ID_SIZE+32+2];
  const char *message;
} OpenROBO_ReadWriteMemory_Data_t;

OpenROBO_ReadWriteMemory_Data_t *OpenROBO_ReadWriteMemory_hashTable;
int OpenROBO_ReadWriteMemory_hashSize;
int OpenROBO_ReadWriteMemory_entries;

int OpenROBO_ReadWriteMemory_init(int size)
{
  int i;
  OpenROBO_ReadWriteMemory_hashSize = size;
  OpenROBO_ReadWriteMemory_hashTable = (OpenROBO_ReadWriteMemory_Data_t*)OpenROBO_malloc(sizeof(OpenROBO_ReadWriteMemory_Data_t) * OpenROBO_ReadWriteMemory_hashSize);
  if (OpenROBO_ReadWriteMemory_hashTable == NULL) {
    return OpenROBO_Return_Error;
  }

  // init
  for (i = 0; i < size; i++) {
    OpenROBO_ReadWriteMemory_hashTable[i].key[0] = '\0';
  }

  OpenROBO_ReadWriteMemory_entries = 0;

  return OpenROBO_Return_Success;
}

void OpenROBO_ReadWriteMemory_realloc(int newSize)
{
    DBGPRINTF("OpenROBO_ReadWriteMemory_realloc: %d -> %d [%d]\n", OpenROBO_ReadWriteMemory_hashSize, newSize, OpenROBO_ReadWriteMemory_entries);
    OpenROBO_ReadWriteMemory_Data_t *oldTable = OpenROBO_ReadWriteMemory_hashTable;
    int oldSize = OpenROBO_ReadWriteMemory_hashSize;
    OpenROBO_ReadWriteMemory_init(newSize);
    int n;
    for (n = 0; n < oldSize; n++) {
        if (oldTable[n].key[0] != '\0') {
            OpenROBO_ReadWriteMemory_put(oldTable[n].key, oldTable[n].message);
        }
    }
    free(oldTable);
}

int OpenROBO_ReadWriteMemory_hash(const char *key)
{
    int n, h = 0;
    for (n = 0; key[n] != '\0'; n++) {
        h = (h * 137 + (key[n]&0xff)) % OpenROBO_ReadWriteMemory_hashSize;
    }
    return h;
}

int OpenROBO_ReadWriteMemory_put(const char *key, const char *message)
{
  double time = (double)clock()/(double)CLOCKS_PER_SEC;
  int n, h = OpenROBO_ReadWriteMemory_hash(key);
  char *new_message = (char *)OpenROBO_malloc(strlen(message)+sizeof(char)*32);
  if (new_message == NULL) {
    DBGPRINTF("Error: OpenROBO_malloc");
    DBGABORT();
    return -1;
  }
  strcpy(new_message, &message[sizeof(OpenROBO_MessageHeader_Write)]);
  OpenROBO_Message_setTime(new_message, time);

  for (n = 0; n < OpenROBO_ReadWriteMemory_hashSize; n++) {
    int ix = (h + n) % OpenROBO_ReadWriteMemory_hashSize;
    if (OpenROBO_ReadWriteMemory_hashTable[ix].key[0] == '\0') {
      // new entry
      strcpy(OpenROBO_ReadWriteMemory_hashTable[ix].key, key);
      OpenROBO_ReadWriteMemory_hashTable[ix].message = new_message;
      OpenROBO_ReadWriteMemory_entries++;
      if (OpenROBO_ReadWriteMemory_entries * 3 > OpenROBO_ReadWriteMemory_hashSize * 2) {
        OpenROBO_ReadWriteMemory_realloc(OpenROBO_ReadWriteMemory_hashSize * 3 / 2);
      }
      return ix;
    } else if (strcmp(OpenROBO_ReadWriteMemory_hashTable[ix].key, key) == 0) {
      // exist, update
      OpenROBO_free((void *)OpenROBO_ReadWriteMemory_hashTable[ix].message);
      OpenROBO_ReadWriteMemory_hashTable[ix].message = new_message;
      return ix;
    }
  }
  DBGPRINTF("Error: OpenROBO_ReadWriteMemory_hash Table Full");
  DBGABORT();
  return -1;
}

const char *OpenROBO_ReadWriteMemory_get(const char *key)
{
  int n, h = OpenROBO_ReadWriteMemory_hash(key);
  for (n = 0; n < OpenROBO_ReadWriteMemory_hashSize; n++) {
    int ix = (h + n) % OpenROBO_ReadWriteMemory_hashSize;
    if (OpenROBO_ReadWriteMemory_hashTable[ix].key[0] == '\0') {
      // not exist
      return NULL;
    } else if (strcmp(OpenROBO_ReadWriteMemory_hashTable[ix].key, key) == 0) {
      // found
      return OpenROBO_ReadWriteMemory_hashTable[ix].message;
    }
  }
  return NULL;
}
