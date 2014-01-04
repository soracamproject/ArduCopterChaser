■■■■■　仕様　■■■■■


■■■■■　改造ファイルと場所　■■■■■
ArduCopter.pde
　"CHASERモード用グローバル変数"の項目を追加
　set_yaw_mode関数内にYAW_CHASERのcaseを追加
　update_yaw_mode関数内にYAW_CHASERのcaseを追加
　set_throttle_mode内にTHROTTLE_AUTO_TAKEOFFを追加（流用）
　update_throttle_mode内にTHROTTLE_AUTO_TAKEOFFを追加（流用）　←以前と違うところ★★★
chaser.pde
　すべて　新規作成ファイル
commands_logic.pde
　do_c_takeoff関数を新規作成
　do_chaser関数を新規作成
config.h
　#define OPTFLOW          ENABLED　→　DISABLED　容量削減のため
　#define CLI_ENABLED     ENABLED　→　DISABLED　容量削減のため
defines.h
　YAW_CHASERを追加　番号は10
　THROTTLE_AUTO_TAKEOFFを追加　番号は6
　Auto Pilot modes
　　C_TAKEOFFモードとCHASERモードを追加　番号は14と15　←以前と番号変わっているので注意★★★
　　NUMをふたつ増やす
　Navigation modes held in nav_mode variableにNAV_CHASERを追加　番号は4
　ファイル末尾にCHASER関連の#defineを追加
GCS_Mavlink.pde
   bool GCS_MAVLINK::stream_trigger(enum streams stream_num)内にCHASERデバッグ用項目追加
   void GCS_MAVLINK::data_stream_send(void)内にCHASERデバッグ用項目追加
　GCS_MAVLINK::handleMessageの末尾にcase MAVLINK_MSG_ID_CHASER_CMDの項目を追加。
   send_nav_controller_outputおよびsend_vfr_hudにCHASERデバッグ用項目追加
navigation.pde
　set_nav_modeのスイッチにNAV_CHASERを追加
　update_nav_modeのスイッチにNAV_CHASERを追加
system.pde
　mode_requires_GPS関数内にC_TAKEOFFとCHASER追加
　set_mode関数内にC_TAKEOFFモードとCHASERモードを追加

==以下Librariesフォルダ内==
ardupilotmega.xml
　末尾にCHASER_CMDを追加
AC_WPNav.h
　CHASER関連define追加
　update_loiter_for_chaserを追加
　get_loiter_position_to_velocity_chaserを追加
AC_WPNav.cpp
　update_loiter_for_chaserを作成　　←以前と大きく変わっている部分
　get_loiter_position_to_velocity_chaserを作成　　←以前と大きく変わっている部分


■■■■■　mavlinkにメッセージを追加する方法　■■■■■
・GCS_MAVLink/message_definitions/ardupilotmega.xmlをいじって、generate.shをシェルスクリプトとして実行する
・generate.shは改行コードがCR+LFになっていてmacのコマンドプロンプトだとエラーをはくので注意
・generate.sh実行時はlibrariesフォルダから「./GCS_Mavlink/generate.sh」を叩く
・mavlinkプログラム（githubより入手）の奥のほうにあるmavgen.pyのフォルダにPATH（PYTHONPATHではなくPATH）が通っていないとエラーを吐くので注意
　例：export PATH=(上位フォルダ)/mavlink/pymavlink/generator:$PATH
・そのままgenerateするとmavlink_conversions.hが生成されてmavlink_helper.hからincludeされ、それがエラーをはくけど、
　ひとまずincludeをやめるとコンパイルは通る。他にも色々違う部分ありそう（3.0.1についてくるやつと比べて行数が違う）だけどそのままやってみる


