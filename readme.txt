■■■■■　概要　■■■■■
ArduCopterを改造しChaserモードという自動追尾モードを追加したソフトウェアです。
改造箇所は「変更箇所一覧.txt」を参照のこと

ベースソフトウェア
・ArduCopter 3.2 rc4 相当
・PX4Firmware　3.2 rc4 相当
・PX4NuttX　3.2 rc4 相当



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