■■■■■　概要　■■■■■
ArduCopterを改造しChaserモードという自動追尾モードを追加したソフトウェアです。
改造箇所は「変更箇所一覧.txt」を参照ください

ベースソフトウェア
・ArduCopter [リリース名：ArduCopter-3.2-apm-px4] [コミットID：c8e0f3e]

コンパイルに必要なソフトウェア
　本家もしくはsoracamprojectのgithubから下記リリース名のものをダウンロード
　・PX4Firmware [リリース名：ArduCopter-3.2-apm-px4] [コミットID：dea2cfd]
　・PX4NuttX [リリース名：ArduCopter-3.2-apm-px4] [コミットID：7a62e35]

■■■■■　利用したい方へのメッセージ　■■■■■


①　現在のCHASERモードは下記のような状態です。
　　*** 現在3.2キャッチアップ中のため以下未検証***
　　・Chaser機能は使用できます。Yaw制御は改善が必要なものの対象を追尾することが可能です。
　　　Gimbal制御も使用可能です。tarot 2D gimbalを取り付けることで動作はします。ただし正しく映るような制御はできておらず要改善です。
　　・あまり速い速度は出せませんが、自動追尾しながらターゲットの周りを周回するCircle Chaserという機能が使えます。
　　　CHASER_CHASEもしくはCHASER_STAYステートにいる際にCHASER_CIRCLEステートに入れてください。
　　　ただしYaw制御が完成しておりません。要改善点です。
　　・Stay中もターゲットを自動追尾します。（コプターがその場静止して空撮するモード）
　　***　未検証ここまで　***

②　Chaserを利用する場合、Mavlinkで所定のコマンドを送信する必要があるため
　　利用するためにはbeacon側のコードを読み解く必要があります。

③　本家のほうで、Android＋DroidPlannerを使ったFollowMe機能の開発がガンガン進んでおります。
　　Chaserは一応本家FollowMeとはなんら関係無く開発しておりますが、
　　本家のほうが高機能になる可能性はかなり高いです。
　　本家でなんとかできると判断したらこちらの開発は終了するかもしれません。


■■■■■　コンパイルの仕方　■■■■■
１．PX4コンパイル環境を構築する
・build環境　https://pixhawk.org/dev/minimal_build_env
・toolchain　https://pixhawk.org/dev/toolchain_installation

２．ArduCopterChaserのフォルダと同じ階層にPX4FirmwareとPX4NuttXを置く
・本家(github.com/diydrones)もしくはsoracamprojectのgithubから指定のリリースのものをダウンロードし
　ファイル名を修正して配置　

３．makeする
（やり方ざっくり）
・terminalでArduCopterChaser/ArduCopterフォルダに入る
・「make configure」と打ちEnter
・「make px4-v2」と打ちEnter
（参考URL）
http://dev.ardupilot.com/wiki/building-px4-with-make/

４．uploadする
・make後「make px4-v2-upload」と打ちEnter
・make前の場合はmakeが始まります

■■■■■　変数名の思想　■■■■■
positionはhomeを起点とした位置。単位は[cm]。lat方向(南北)がx、lon方向(東西)がy。
locationは緯度経度。緯度がlat、経度がlon。

■■■■■　その他　■■■■■
プロジェクトHP：http://soracamproject.com
プロジェクトFacebookページ：　https://www.facebook.com/soracamproject
Youtube：https://www.youtube.com/watch?v=R6rmXm6TPVY
