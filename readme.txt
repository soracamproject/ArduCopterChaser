■■■■■　変更履歴　■■■■■

2013.1.4
・CHASER仕様
　・MavlinkでCHASER_CMDが送られてくるとモード遷移する。ただし、予めdefineした範囲外の緯度経度が飛んできたら除外する。
　・CHASERモードはTHROTTLE AUTO, ROLLPITCH AUTO, YAW_CHASER, NAV_CHASERである
　・NAV_CHASERの仕様は以下。
　　・予測制御
　　　・ビーコン位置を指定回平均してなました位置をbeacon_loc_relaxedとする（デフォルト値は5）
　　　・beacon_loc_relaxedが更新された際、前回速度と更新周期を元に次更新時位置destinationを予測する。
　　　　また、その時のtarget位置をoriginとし、destinationと更新周期からtargetの目標速度dest_velを計算する。
　　　　また、dest_velにリミットを設ける（絶対値でかけて比例倍する、つまり方角は変わらない）
　　・現状速度の変化量は加速度で決める。
　　・target位置がorigin-destination間を「目標速度×所定時間（100msくらい？）」超えたら目標速度を0とする
　　・不感帯制御
　　　・ビーコン位置変化量が半径1.5m以内が5回続いたらビーコン位置をラッチする
　　・専用WP_Nav
　　　・基本機能は同じ
　・YAW_CHASERの仕様は以下。
　　・機首の向きはtarget位置に向ける。その変化速度にCHASER_YAW_SLEW_RATEで制限をかける。
　　・targetの更新はupdate_chaserで行われ（10Hz）、機首の目標値更新はupdate_yaw_modeで行われる（100Hz）
　・その他変更点
　　・GPSが使えない場合はStabilizeに移行する（failsafe）

・BEACON仕様
　・起動したら所定時間後にARM→テイクオフ→CHASERへと移行する
　・CHASER移行後は約0.7sec毎にGPSをアップデートし、CHASER_CMDを送信する

・デバッグ用変数割付表
　CHASER_DEBUGをdefineすると下記設定が有効になる
　　rate設定はSTREAM_POSITIONで50Hz設定、それ以外は無効
　　MSGとしては、「MSG_NAV_CONTROLLER_OUTPUT」と「MSG_VFR_HUD」
　　mavlink関数としては、「mavlink_msg_nav_controller_output_send」と「mavlink_msg_vfr_hud_send」
　　変更箇所は、GCS_Mavlink.pdeの例えば、static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)内を編集する
　　変数の型については、例えばmavlink_msg_nav_controller_output_send.hに記載。GCS_Mavlink.pdeに転記済。

　　　　型				元変数　　   　　　割付変数　　備考
　nav_controller
　　　　float           nav_roll              chaser_copter_pos.x
　　　　float           nav_pitch           chaser_copter_pos.y
　　　　float           alt_error             chaser_target.x
　　　　float           aspd_error         chaser_target.y
　　　　float           xtrack_error
　　　　int16_t        nav_bearing
　　　　int16_t        target_bearing
　　　　uint16_t      wp_dist
　vfr_hud
　　　　float            airspeed           chaser_destination.x
　　　　float            groundspeed    chaser_destination.y
　　　　float            alt
　　　　float            climb
　　　　int16_t        heading
　　　　uint16_t      throttle


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
　　C_TAKEOFFとCHASERとC_LANDを追加　番号は14と15と16　←以前と番号変わっているので注意★★★
　　NUM_MODESを増やす
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
　set_mode関数内にC_TAKEOFFモードとCHASERモードとC_LANDモードを追加

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

==以下Beacon関連==
libraries/FastSerial
　DLしてきたものそのまま
libraries/BC_Compat
　新規作成

■■■■■　mavlinkにメッセージを追加する方法　■■■■■
・GCS_MAVLink/message_definitions/ardupilotmega.xmlをいじって、generate.shをシェルスクリプトとして実行する
・generate.shは改行コードがCR+LFになっていてmacのコマンドプロンプトだとエラーをはくので注意
・generate.sh実行時はlibrariesフォルダから「./GCS_Mavlink/generate.sh」を叩く
・mavlinkプログラム（githubより入手）の奥のほうにあるmavgen.pyのフォルダにPATH（PYTHONPATHではなくPATH）が通っていないとエラーを吐くので注意
　例：export PATH=(上位フォルダ)/mavlink/pymavlink/generator:$PATH
・そのままgenerateするとmavlink_conversions.hが生成されてmavlink_helper.hからincludeされ、それがエラーをはくけど、
　ひとまずincludeをやめるとコンパイルは通る。他にも色々違う部分ありそう（3.0.1についてくるやつと比べて行数が違う）だけどそのままやってみる


