■■■■■　概要　■■■■■
ArduCopterを改造しChaserモードという自動追尾モードを追加したソフトウェアです。
改造箇所は「変更箇所一覧.txt」を参照のこと

ベースソフトウェア
・ArduCopter 3.2 rc5
・PX4Firmware　3.2 rc4相当　←本家では無くsoracamprojectのgithubから最新リリースをDL願います
・PX4NuttX　3.2 rc4相当 　←本家では無くsoracamprojectのgithubから最新リリースをDL願います

■■■■■　利用したい方へのメッセージ　■■■■■
①　現在、CHASERモードが"一部"ちゃんと動きません。
　　YAWコントロールとGIMBALコントロール以外はCHASERできます。
　　ですが、Mavlinkで所定のコマンドを送信する必要があるため
　　利用するためにはbeacon側のコードを読み解く必要があります。
②　本家のほうで、Android＋DroidPlannerを使ったFollowMe機能の開発がガンガン進んでおります。
　　Chaserは一応本家FollowMeとはなんら関係無く開発しておりますが、
　　本家のほうが高機能になる可能性はかなり高いです。
　　本家でなんとかできると判断したらこちらの開発は終了するかもしれません。


■■■■■　コンパイルの仕方　■■■■■
１．PX4コンパイル環境を構築する
・build環境　https://pixhawk.org/dev/minimal_build_env
・toolchain　https://pixhawk.org/dev/toolchain_installation

２．ArduCopterChaserのフォルダと同じ階層にPX4FirmwareとPX4NuttXを置く
・https://github.com/soracamproject/PX4Firmware　から最新リリースをzip等でダウンロードし解凍してファイル名を修正し配置
・https://github.com/soracamproject/PX4NuttX　　も同様
※本家(github.com/diydrones)からDLするとバージョンが合わない場合があるため注意

３．makeする
（やり方ざっくり）
・terminalでArduCopterChaser/ArduCopterフォルダに入る
・「make configure」と打ちEnter
・「make px4-v2」と打ちEnter
（参考URL）
http://dev.ardupilot.com/wiki/building-px4-with-make/

４．uploadする
省略


■■■■■　その他　■■■■■
プロジェクトHP：http://soracamproject.com
