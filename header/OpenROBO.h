#ifndef __OPENROBO_H__
#define __OPENROBO_H__

#include <stdint.h>

/**
* The origin is TinyCThread
* TinyCThread (https://tinycthread.github.io)
* Copyright (c) 2012 Marcus Geelnard
* Copyright (c) 2013-2016 Evan Nemerson
* under the zlib/libpng license
*
* @file
* @mainpage TinyCThread API Reference
*
* @section intro_sec Introduction
* TinyCThread is a minimal, portable implementation of basic threading
* classes for C.
*
* They closely mimic the functionality and naming of the C11 standard, and
* should be easily replaceable with the corresponding standard variants.
*
* @section port_sec Portability
* The Win32 variant uses the native Win32 API for implementing the thread
* classes, while for other systems, the POSIX threads API (pthread) is used.
*
* @section misc_sec Miscellaneous
* The following special keywords are available: #_Thread_local.
*
* For more detailed information, browse the different sections of this
* documentation. A good place to start is:
* tinycthread.h.
*/

/* Which platform are we on? */
#if defined(_WIN32) || defined(__WIN32__) || defined(__WINDOWS__)
	#define _OPENROBO_WIN32_
#else
	#define _OPENROBO_POSIX_
#endif

enum {
  OpenROBO_MessageType_Start = 0,
  OpenROBO_MessageType_Stop,
  OpenROBO_MessageType_Wait,
  OpenROBO_MessageType_Return,
  OpenROBO_MessageType_Read,
  OpenROBO_MessageType_Write,
};

// thread
#if defined(_OPENROBO_WIN32_)

#include <windows.h>
typedef HANDLE OpenROBO_Thread_t;

#else

#include <pthread.h>
typedef pthread_t OpenROBO_Thread_t;

#endif

#define OpenROBO_malloc malloc
#define OpenROBO_free free

#define OPENROBO_DEFAULT_ACCEPT_PORT 50002

#define OPENROBO_SUBSYSTEM_ID_SIZE 128
#define OPENROBO_FUNCTION_NAME_SIZE 256
#define OPENROBO_THREAD_ID_SIZE (OPENROBO_SUBSYSTEM_ID_SIZE+OPENROBO_FUNCTION_NAME_SIZE)

#define OPENROBO_IP_STR_LEN 15
#define OPENROBO_PORT_STR_LEN 5
#define OPENROBO_AGENTS_COMMECTION_MAX 16

enum {
  OpenROBO_Return_FailToInit = -9,
  OpenROBO_Return_NoValue = -8,
  OpenROBO_Return_NotUpdated = -7,
  OpenROBO_Return_NonConnection = -6,
  OpenROBO_Return_ExitedThread = -5,
  OpenROBO_Return_DoubleCreateSubthread = -4,
  OpenROBO_Return_BufferOver = -3,
  OpenROBO_Return_Disconnected = -2,
  OpenROBO_Return_Error = -1,
  OpenROBO_Return_Success = 0,
};

typedef void (*OpenROBO_Thread_Start_t)(void *);
typedef void (*OpenROBO_MessageFunction_t)(const char *);
typedef int (*OpenROBO_SubthreadFunction_t)(int, char *[]);
typedef int (*OpenROBO_InitFunction_t)();

typedef struct {
  OpenROBO_MessageFunction_t func;
  char name[OPENROBO_FUNCTION_NAME_SIZE];
} OpenROBO_MessageFunctionEntry_t;

typedef struct {
  OpenROBO_SubthreadFunction_t func;
  char name[OPENROBO_FUNCTION_NAME_SIZE];
} OpenROBO_SubthreadFunctionEntry_t;

typedef struct {
  OpenROBO_InitFunction_t func;
} OpenROBO_InitFunctionEntry_t;

#define OPENROBO_END_OF_MESSAGE_FUNCTION_ENTRY {NULL,""}
#define OPENROBO_END_OF_SUBTHREAD_FUNCTION_ENTRY {NULL,""}
#define OPENROBO_END_OF_INIT_FUNCTION_ENTRY {NULL}

extern const char* const OpenROBO_SubsystemName_TASKPLANNER;

/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */
/**
 *  メインスレッドの初期化を行う。
 *  初期化として、スレッドやソケットのための初期化やメモリの確保を行う。
 *  main関数の初めに必ず1回この関数を呼ぶ。
 *
 *  @param[in] subsystemName 自身のサブシステム名。例えば"TP","VS","AC"など。
 *
 */
int OpenROBO_StartupMainThread(const char* subsystemName);

/**
 * メインスレッドで次のメインループを実行
 * (1) メッセージ受信待ち
 * (2) 受信したメッセージの種類を判別; Operation, Wait, Stop, Read, Write
 * (3) 種類に応じた処理を行う
 *  (3-1) Operation: オペレーションスレッドを作り、メッセージに対応したロボット動作関数を実行
 *  (3-2) Wait: オペレーションスレッドからのリターンを待つ
 *  (3-3) Stop: オペレーションスレッドへ終了要求を出す
 *  (3-4) Read: メモリ上から対応する値を含むメッセージを取り出す
 *  (3-5) Write: メモリ上へ対応する値を含むメッセージを格納する
 *    (1)へ戻る(ループ)
 *
 * @param[IN] operationEntry Operation Messageで呼び出される関数の名前と関数ポインタ(エントリ)のリスト
 */
int OpenROBO_Main(OpenROBO_MessageFunctionEntry_t operationEntry[]);

/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
   OpenROBO_Thread
   The origin is TinyCThread

   TinyCThread (https://tinycthread.github.io)
   Copyright (c) 2012 Marcus Geelnard
   2013-2016 Evan Nemerson
   under the zlib/libpng license
   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */

/**
 * TODO: 削除予定 -> 初期化処理へ統合
 *
 * HRIのGUIのスレッド、CGのCG表示のスレッドといったサブシステム固有のスレッドを作る
 *
 * @param[IN] func スレッドとなる関数のエントリ(関数ポインタ)
 * @param[IN] threadName スレッド名(関数名)
 * @param[IN] argc スレッドとなる関数に引数として渡される
 * @param[IN] argv スレッドとなる関数に引数として渡される
 *
 */
int OpenROBO_Thread_CreateSubthread(int (*func)(int, char *[]), const char* funcName, int argc, char *argv[]);

/**
 *  オペレーションスレッドにおいて、終了要求がきているかを調べる
 *
 * @retval ==0 終了要求がきている
 * @retval !=0 終了要求がきていない
 */
int OpenROBO_CheckWorking(void);

/**
 *  オペレーションスレッドにおいて、終了要求が来るまで待つ
 */
int OpenROBO_WaitForStopMessage(void);

/**
 * オペレーションスレッドを作る
 *
 * @param[in] func メッセージ関数
 * @param[int] message メッセージ関数に渡すメッセージ
 *
 */
int OpenROBO_Thread_CreateOperationThread(OpenROBO_MessageFunction_t func, char* message);

/**
 * TODO: 名前変更予定
 *
 * スレッドへ終了要求のメッセージを送る
 *
 * @param[IN] destionationID 送り先のサブシステム名
 * @param[IN] functionName 終了する関数名
 */
int OpenROBO_RequestToExitThread(const char* destionationID, const char* functionName);

/**
 * TODO: 名前変更予定
 *
 * スレッドの終了待ちのメッセージを送る
 *
 * @param[IN] destionationID 送り先のサブシステム名
 * @param[IN] functionName 終了を待つ関数名
 * @param[OUT] returnMessage 返答メッセージ
 */
int OpenROBO_JoinThread(const char* destionationID, const char* functionName, char **returnMessage);

/**
 * TODO: 名前変更予定
 *
 * スレッドへ終了要求のメッセージを送り、終了を待つ
 *
 * @param[IN] destionationID 送り先のサブシステム名
 * @param[IN] functionName 終了する関数名
 * @param[OUT] returnMessage 返答メッセージ
 */
int OpenROBO_ExitThread(const char* destionationID, const char* functionName, char **returnMessage);

/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

   OpenROBO_Socket

   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */

/**
 * 接続要求受付とメッセージの受信を行う
 * どちらも来ていない場合はこの関数は処理を返さず停止する(blocking関数)
 *
 * @param[out] message 受信したメッセージ
 */
int OpenROBO_Socket_ReceiveMessage(char** message);

/**
 * 返答メッセージ(Return Message)を受診する
 *
 * @param[in] 受信元のサブシステム名
 * @param[out] message 受信したメッセージ
 */
int OpenROBO_Socket_ReceiveReturnMessage(const char* sourceID, char** message);

/**
 * ロボット動作関数実行の指示を出すOperation Messageを送信する
 *
 * @param[in] destionationID 送信先のエージェント名
 * @param[in] message メッセージ
 */
int OpenROBO_Socket_SendCommandMessage(const char* destinationID, char* message);

/**
 * メッセージに対する返答のReturn Messageを送る
 *
 * @param[in] returnMessage Return Message
 */
int OpenROBO_Socket_SendReturnMessage(const char *returnMessage);

/**
 * メッセージを受信する
 *
 * @param[in] destionationID 受信元のエージェント名
 * @param[in] message メッセージ
 * @param[in] messageSize メッセージのサイズ(受信の最大値)
 * @param[out] recvSize 受信したメッセージのサイズ
 */
int OpenROBO_Socket_RecvMessage(const char *sourceID,char *message,int messageSize,int *recvSize);

/**
 * エージェント間の接続情報を交換して接続できるようにする
 * タスクプランナ以外のエージェントでのみ使用する
 *
 * まず、タスクプランナへ接続して自身の接続情報を送り、
 * その後、タスクプランナから他のエージェントの接続情報を受け取る。
 *
 * @param[in] ip タスクプランナのIPアドレス
 * @param[in] port タスクプランナのポート番号
 */
int OpenROBO_Socket_MakeConnection(const char *ip,uint16_t port);

/*
 * エージェント間の接続情報を交換して接続できるようにする
 * タスクプランナでのみ使用する
 *
 * まず、タスクプランナ以外のエージェントからの接続を受け付けて、それぞれの接続情報を受け取り、
 * その後、タスクプランナは他のエージェントへの接続情報を送る。
 *
 * @param[in] port 接続を受け付けるポート番号
 * @param[in] ids 接続を受け付けるエージェント一覧(const char* const ids[] = {"ARMCONTROLLER", "VISION", NULL};のように文字列の配列で、最後はNULLで終わるようなフォーマット)
  */
int OpenROBO_Socket_AcceptConnection(uint16_t port, const char* const ids[]);


/* _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

   OpenROBO_Message

   _/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/ */

/**
 * メッセージから送り元のエージェント名を取得
 *
 * @param[in] message メッセージ
 * @param[out] sourceID 送り元のエージェント名
 */
void OpenROBO_Message_GetSourceID(const char* message, const char** sourceID);

/**
 * メッセージから送り先のエージェント名を取得
 *
 * @param[in] message メッセージ
 * @param[out] destinationID 送り先のエージェント名
 */
void OpenROBO_Message_GetDestinationID(const char* message, const char** destinationID);

/**
 * メッセージからSubjectを取得
 *
 * @param[in] message メッセージ
 * @param[out] subject subject
 */
void OpenROBO_Message_GetSubject(const char* message, const char** subject);

/**
 * Read/Write Messageの値の最終更新時間を取得
 *
 * @param[in] message メッセージ
 * @param[out] double 時間
 */
void OpenROBO_Message_GetTime(const char* message, double* time);

/**
 * メッセージからパラメータの値(文字列)を取り出す
 * (メッセージから内部コードへ変換)
 * 
 * @param[in] message メッセージ
 * @param[in] name パラメータ名
 * @param[out] str 取り出した文字列
 */
void OpenROBO_Message_GetParam_string(const char *message, const char *name, const char **str);

/**
 * メッセージからパラメータの値(T-Matrix(4x4の同次変換行列))を取り出す
 * (メッセージから内部コードへ変換)
 * 
 * @param[in] message メッセージ
 * @param[in] name パラメータ名
 * @param[out] TMatrix 取り出したT-Matrix
 */
void OpenROBO_Message_GetParam_TMatrix(const char *message, const char *name, double TMatrix[4][4]); 

/**
 * メッセージからパラメータの値(double型)を取り出す
 * (メッセージから内部コードへ変換)
 * 
 * @param[in] message メッセージ
 * @param[in] name パラメータ名
 * @param[out] value 取り出した値
 */
void OpenROBO_Message_GetParam_double(const char *message, const char *name, double *value); 

/**
 * メッセージからパラメータの値(double型配列)を取り出す
 * (メッセージから内部コードへ変換)
 * 
 * @param[in] message メッセージ
 * @param[in] name パラメータ名
 * @param[out] values 取り出した配列
 * @param[in] n 取り出した配列の要素数
 */
void OpenROBO_Message_GetParam_doubleArray(const char *message,const char *name, double *values, unsigned int n);

/**
 * メッセージからパラメータの値(int型)を取り出す
 * (メッセージから内部コードへ変換)
 * 
 * @param[in] message メッセージ
 * @param[in] name パラメータ名
 * @param[out] value 取り出した値
 */
void OpenROBO_Message_GetParam_int(const char *message, const char *name, int *value); 

/**
 * メッセージからパラメータの値(int型配列)を取り出す
 * (メッセージから内部コードへ変換)
 * 
 * @param[in] message メッセージ
 * @param[in] name パラメータ名
 * @param[out] values 取り出した配列
 * @param[in] n 取り出した配列の要素数
 */
void OpenROBO_Message_GetParam_intArray(const char *message,const char *name, int *values, unsigned int n);


/**
 * メッセージからパラメータの値(byte列(char型配列))を取り出す
 * (メッセージから内部コードへ変換)
 * 
 * @param[in] message メッセージ
 * @param[in] name パラメータ名
 * @param[out] values 取り出した配列
 * @param[in] n 取り出した配列の要素数
 */
void OpenROBO_Message_GetParam_byteArray(const char *message,const char *name, unsigned char *values, unsigned int n);

/**
 * メッセージから戻り値を取り出し、メッセージのメモリを解放する
 * (メッセージから内部コードへ変換)
 *
 * @param[in] message メッセージ(この関数を呼び出し後解放される)
 * @param[out] 取り出した戻り値
 */
void OpenROBO_Message_GetReturnValue(const char *message, int *value);

/**
 * メッセージのバッファを取得する
 * @param[out] バッファを返すアドレス
 * @retval ==0 成功
 * @retval !=0 失敗
 */
int OpenROBO_Message_GetBuffer(char** message);

/**
 * 文字列のパラメータをメッセージ形式に変換し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *  
 * @param[out] message パラメータを変換したメッセージが追記されるメッセージ
 * @param[in] name パラメータ名
 * @param[in] str 文字列 
 */
void OpenROBO_Message_SetParam_string(char *message, const char *name, const char *str); 


/**
 * T-Matrix(4x4の同次変換行列)のパラメータをメッセージ形式に変換し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *  
 * @param[out] message パラメータを変換したメッセージが追記されるメッセージ
 * @param[in] name パラメータ名
 * @param[in] TMatrix T-Matrix
 */
void OpenROBO_Message_SetParam_TMatrix(char *message, const char *name, const double TMatrix[4][4]); 

/**
 * double型のパラメータをメッセージ形式に変換し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *  
 * @param[out] message パラメータを変換したメッセージが追記されるメッセージ
 * @param[in] name パラメータ名
 * @param[in] value パラメータの値
 */
void OpenROBO_Message_SetParam_double(char *message, const char *name, const double *value); 

/**
 * double型配列のパラメータをメッセージ形式に変換し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *  
 * @param[out] message パラメータを変換したメッセージが追記されるメッセージ
 * @param[in] name パラメータ名
 * @param[in] values パラメータの配列
 * @param[in] n パラメータ配列の要素数
 */
void OpenROBO_Message_SetParam_doubleArray(char *message,const char *name,const double *values, unsigned int n);

/**
 * int型のパラメータをメッセージ形式に変換し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *  
 * @param[out] message パラメータを変換したメッセージが追記されるメッセージ
 * @param[in] name パラメータ名
 * @param[in] value パラメータの値
 */
void OpenROBO_Message_SetParam_int(char *message, const char *name, const int *value); 

/**
 * int型配列のパラメータをメッセージ形式に変換し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *  
 * @param[out] message パラメータを変換したメッセージが追記されるメッセージ
 * @param[in] name パラメータ名
 * @param[in] values パラメータの配列
 * @param[in] n パラメータ配列の要素数
 */
void OpenROBO_Message_SetParam_intArray(char *message,const char *name,const int *values, unsigned int n);

/**
 * byteデータ列(char型配列)のパラメータをメッセージ形式に変換し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *  
 * @param[out] message パラメータを変換したメッセージが追記されるメッセージ
 * @param[in] name パラメータ名
 * @param[in] values パラメータの配列
 * @param[in] n パラメータ配列の要素数
 */
void OpenROBO_Message_SetParam_byteArray(char *message,const char *name,const unsigned char *values, unsigned int n);

/**
 * 戻り値をメッセージ形式に変換し、それを第1引数で渡したメッセージに追記する
 * (内部コードからメッセージに変換)
 *  
 * @param[out] message 戻り値を変換したメッセージが追記されるメッセージ
 * @param[in] values 戻り値
 */
void OpenROBO_Message_SetReturnValue(char *message, int value);

/**
 * メッセージにSubjectをセット
 *
 * @param[out] message メッセージ
 * @param[in] subject subject
 */
void OpenROBO_Message_SetSubject(char* message, const char* subject);

/**
 * メッセージに指定のパラメータ名が含まれているか
 *
 * @param[in] message メッセージ
 * @param[in] name パラメータ名
 *
 * @retval true(!=0) 含まれている
 * @retval false(==0) 含まれていない
 */
int OpenROBO_Message_HasParam(const char* message, const char* name);

/**
 *  渡されたバッファにOperation Messageを作る
 * @param[out] message メッセージのバッファ
 * @param[in] subject オペレーション名
 */
void OpenROBO_Message_MakeOperationMessage(char *message, const char* subject);

/**
 *  渡されたバッファにWait Messageを作る
 * @param[out] message メッセージのバッファ
 * @param[in] subject 停止を待つオペレーション名
 */
void OpenROBO_Message_MakeWaitMessage(char *message, const char* subject);

/**
 *  渡されたバッファにStop Messageを作る
 * @param[out] message メッセージのバッファ
 * @param[in] subject 停止するオペレーション名
 */
void OpenROBO_Message_MakeStopMessage(char *message, const char* subject);

/**
 *  渡されたバッファにReturn Messageを作る
 * @param[out] message メッセージのバッファ
 * @param[in] subject 返信元のオペレーション名
 */
void OpenROBO_Message_MakeReturnMessage(char *message, const char* subject);

/**
 *  渡されたバッファにRead Messageを作る
 * @param[out] message メッセージのバッファ
 * @param[in] subject 読み込む値の名称(構造体名)
 */
void OpenROBO_Message_MakeReadMessage(char *message, const char* subject);

/**
 *  渡されたバッファにWrite Messageを作る
 * @param[out] message メッセージのバッファ
 * @param[in] subject 書き込む値の名称(構造体名)
 */
void OpenROBO_Message_MakeWriteMessage(char *message, const char* subject);

int OpenROBO_Message_GetMessageType(const char *message);

#endif // __OPENROBO_H__
