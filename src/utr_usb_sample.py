#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# [[[注意事項]]]
# すべての条件分岐を網羅（確認）しているわけではありません
# 作りこみには、当社の製品の基礎的な知識が必要です
# 該当製品のプロトコル仕様書も理解する必要があります
# 機器の設定には、別途RWManagerを使用し、
# 実際の動作における通信ログなどを確認ください

#
#UTR-S201 シリーズ
#サンプルプログラム（無保証） V1.1.5
#USBシリアル接続用(Windows 10でも確認済み)
#Python with Raspberry Pi4
#Raspbian GNU/Linux 11 (bullseye)
#2024/02/06
#
# 事前にpyserialのインストール必要
#
# Revised :  def communicate(command, timeout=1) 関数変更
#            受信データ内部のSTX,ETX,SUM,CRのチェックを実施
#            タイムアウト or ACK/NACK受信 にて、ループを抜ける
#
#            集計結果をファイル保存（追加モード）
#
#            serをglobalからローカルへ変更、引数に設定。
#            詳細コマンド確認を追加(ROMバージョンチェックのみ)
#            変数（小文字）と定数（大文字）に変更
#
#            定数名を変更（CMD_LOCATION、DETAIL_LOCATION、OUTPUT_CH_FREQ_LIST）
#
# Todo : ACK/NACKの判断をする別関数があっても良いかも
# Todo : 文字入力も別関数でチェック（処理）しても良いかも
# Todo : 送信出力の変更ができれば良いかも
# Todo : 連続インベントリモードへの対応（受信のみ。パケット途中からとか）
#        現状の関数だとACK/NACK受信しないと戻らない（コマンドモードのみ対応）
# Todo : アンテナ選択、設定変更などをどうするか
# Todo : OUTPUT_CH_FREQ_LISTを辞書型にするか
# Todo : レスポンスデータの詳細コマンドまで確認するようにすべき
# Todo : 経過時間の取得方法の検討（差分がマイナスになり正しく時間計測できない場合の検討）
#
#
# R/W(リーダライタ)の各種動作設定については、
# Windows版 UTR-RWManagerを使用してください。
#
#
#
# シリアル通信コマンドの詳細は、通信プロトコル仕様書を熟読のこと
# https://www.product.takaya.co.jp/rfid/download/uhf.html/
#
# [コマンド、レスポンスの基本フォーマット]
#  STX(02h) 1バイト #Start Text
#  アドレス  1バイト (RWのIDなど、デフォルトは 00h)
#  コマンド  1バイト
#  データ長  1バイト
#  データ部  0～255バイト
#  ETX(03h) 1バイト  #End Text
#  SUM      1バイト　STXからETXまで
#  CR(0Dh)  1バイト  '/r' キャリッジリターン



'''
# プログラムの設計フロー(Markdown形式)
## ステップ
1. **開始**
2. **モジュールインポート**
    - `serial`, `sys`, `time` など
3. **定数と変数の初期化**
    - コマンド、ボーレート、デバイスリストなど
4. **利用可能デバイスの表示**
    - シリアルポートの一覧を表示
5. **デバイス選択**
    - ユーザーにデバイスを選ばせる
6. **シリアルポート設定**
    - 選ばれたデバイスでシリアルポートを開く
7. **通信確認**
    - ROMバージョンチェックコマンドで通信確認
8. **コマンドモード切り替え**
9. **送信出力と周波数チャンネルの読み取り**
10. **インベントリパラメータの設定**
    - GETとSET
11. **インベントリ(タグ読み取り)回数の指定**
    - ユーザーに繰り返し回数を入力させる
12. **ループ開始（指定回数分）**
    - **タグ読み取り（Inventoryコマンド）**
    - **データ解析**
    - **ブザー制御**
    - **結果表示**
13. **ループ終了**
14. **結果の集計と表示、集計結果保存**
15. **シリアルポートクローズ**
16. **終了**
'''

# 関連モジュールをインポート
import sys
import time
import datetime
import re

import serial
from   serial.tools import list_ports

from   typing       import List, Optional


# UTR用 シリアル送信コマンドの定義
COMMANDS = {
    # ROMバージョンの読み取りコマンド
    'ROM_VERSION_CHECK': bytes([0x02, 0x00, 0x4F, 0x01, 0x90, 0x03, 0xE5, 0x0D]),
    # コマンドモードへの切り替えコマンド
    'COMMAND_MODE_SET': bytes([0x02, 0x00, 0x4E, 0x07, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0x6A, 0x0D]),
    # UHF_Inventoryコマンド
    'UHF_INVENTORY': bytes([0x02, 0x00, 0x55, 0x01, 0x10, 0x03, 0x6B, 0x0D]),
    # UHF_GetInventoryParam（RFタグ読み取り時のインベントリ処理に使用するパラメータの取得をおこなうコマンド)
    # コマンドモード用パラメータ
    'UHF_GET_INVENTORY_PARAM':bytes([0x02,0x00,0x55,0x02,0x41,0x00,0x03,0x9D,0x0D]),
    # UHF_SetInventoryParamコマンド(必要に応じて変更、別途関数あり)
    # コマンドモード用パラメータ
    'UHF_SET_INVENTORY_PARAM': bytes([0x02,0x00,0x55,0x09,0x30,0x00,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x14,0x0D]),
    # UHF送信出力設定読み取りコマンド
    # コマンドモード用パラメータ
    'UHF_READ_OUTPUT_POWER': bytes([0x02,0x00,0x55,0x03,0x43,0x01,0x00,0x03,0xA1,0x0D]),
    # UHF送信周波数チャンネル読み取りコマンド
    # コマンドモード用パラメータ
    'UHF_READ_FREQ_CH': bytes([0x02,0x00,0x55,0x03,0x43,0x02,0x00,0x03,0xA2,0x0D]),
    # UHFブザー制御コマンド:応答要求＋ピー音
    'UHF_BUZZER_pi': bytes([0x02,0x00,0x42,0x02,0x01,0x00,0x03,0x4A,0x0D]),
    # UHFブザー制御コマンド:応答要求＋ピッピッピ音
    'UHF_BUZZER_pipipi': bytes([0x02,0x00,0x42,0x02,0x01,0x01,0x03,0x4B,0x0D]),
    # UHF書き込みコマンド（一例、使ってません）
    'UHF_WRITE': bytes([0x02,0x00,0x55,0x08,0x16,0x01,0x00,0x00,0x00,0x02,0x04,0x56,0x03,0xD5,0x0D]),
}

# 定数の定義
HEADER_LENGTH     = 4        # STX, アドレス, コマンド, データ長 (各1バイト)
FOOTER_LENGTH     = 3        # ETX, SUM, CR (各1バイト)
STX : bytes       = b'\x02'  # Start Text
ADD : bytes       = b'\x00'  # RW IDなど(設定により変化することあり)
ETX : bytes       = b'\x03'  # End Test
CR  : bytes       = b'\x0D'  # '/r' キャリッジリターン
ACK : bytes       = b'\x30'  # ACK (Acknowledgment)
NACK: bytes       = b'\x31'  # NACK (Negative Acknowledgment):
INV : bytes       = b'\x6C'  # インベントリコマンド
BUZ : bytes       = b'\x42'  # ブザーコマンド
CMD_LOCATION      = 2        # コマンドの位置（3バイト目）
DETAIL_LOCATION   = 4        # 詳細コマンド位置（5バイト目）
DETAIL_ROM: bytes = b'\x90'  # ROMバージョン読み取り詳細コマンド
DETAIL_INV: bytes = b'\x10'  # インベントリ詳細コマンド

OUTPUT_CH_FREQ_LIST = [916.0, 916.2, 916.4, 916.6, 916.8, 917.0, 917.2, 917.4, 917.6, 917.8, 918.0, 918.2, 918.4, 918.6, 918.8, 919.0, 919.2, 919.4, 919.6, 919.8, 920.0, 920.2, 920.4, 920.6, 920.8, 921.0, 921.2, 921.4, 921.6, 921.8, 922.0, 922.2, 922.4, 922.6, 922.8, 923.0, 923.2, 923.4]

#【シリアルデータ通信関数】
# データ送信後に、受信データ解析を実施
# 制御フロー参照:
# https://www.product.takaya.co.jp/dcms_media/other/TDR-OTH-PROGRAMMING-103.pdf
#
# タイムアウトもしくは、ACK, NACKを受信したらループを抜ける
#
# 受信データが多い(たくさんのタグ:30枚以上を読み取りする場合など)は、
# タイムアウト時間を多くする必要あり。
# もしくはタイムアウト処理の方法を変更する。
def communicate(ser, command, timeout = 1):
    """
    受信データを1バイトずつ解析(STX, CR, ETX, SUM)しながら
    正常なレスポンスコマンドのみを戻す。
    :param ser:     シリアル通信情報を取得
    :param command: 送信データ(上位 --> R/W)
    :param timeout: タイムアウト(デフォルト1秒)
    :return complete_response: 正常なレスポンスコマンド
    """
    complete_response  = b'' # 解析後の正常レスポンス
    receive_buffer = []      # BUF 受信データバッファー
    buffer_length = 0        # B_LEN
    data_length = 0          # D_LEN

    if ser is not None:
        #コマンド送信(上位 -> RW)
        ser.write(command)

    #タイム計測開始（現在の時刻を取得）
    start_time = time.time()

    #シリアル受信、データ解析処理
    while True:
        # タイムアウト処理
        if (time.time() - start_time) > timeout:
            print("タイムアウト: レスポンスが一定時間内に受信されませんでした。")
            return complete_response

        if ser is not None:
            # 受信バッファにデータがあれば、
            # 1バイトずつ読み取り
            receive_buffer += ser.read(1)
            buffer_length = len(receive_buffer) #バッファーの長さを取得
            #print(receive_buffer) # デバッグ用

        # receive_buffer内になにかデータがあり、先頭がSTX(0x02)だったら、
        if receive_buffer:
            if receive_buffer[0] == STX[0]:
            #print("receive_buffer=STX") # デバッグ用

                # バッファ長がヘッダー長(4バイト)以上であれば、
                if buffer_length >= HEADER_LENGTH:
                    # データ長(4バイト目)を data_lengthに入力
                    data_length = receive_buffer[HEADER_LENGTH - 1]

                    #print("D len", data_length) # デバッグ用

                    # バッファ長が (データ長+ヘッダ長+フッタ長)以上であれば、
                    if buffer_length >= (data_length + HEADER_LENGTH + FOOTER_LENGTH):
                        #print("length ok") # デバッグ用

                        # 最後位がCR(0x0D)であれば、
                        if receive_buffer[(data_length + HEADER_LENGTH + FOOTER_LENGTH) - 1] == CR[0]:
                            #print("same CR") # デバッグ用

                            # ETXの位置にETX(0x03)があれば、
                            if receive_buffer[data_length + HEADER_LENGTH] == ETX[0]:
                                #print("same ETX") # デバッグ用

                                # SUM値の確認がTrueであれば、
                                if verify_sum_value(receive_buffer[:(data_length + HEADER_LENGTH + FOOTER_LENGTH)]):
                                    #print("SUM OK") # デバッグ用
                                    # 戻り値に、フォーマット確認済みのバッファを追加
                                    complete_response += bytes(receive_buffer[:(data_length + HEADER_LENGTH + FOOTER_LENGTH)])
                                    #print(complete_response.hex()) # デバッグ用

                                    # ACK, NACK受信していたら抜ける
                                    #print("ACK NACK", receive_buffer[2]) # デバッグ用
                                    if receive_buffer[CMD_LOCATION] in [ACK[0], NACK[0]]:
                                        #print("ACK NACK") # デバッグ用
                                        receive_buffer = [] # バッファをクリア
                                        return complete_response

                                    # 確認済みバッファをクリア
                                    receive_buffer = receive_buffer[(data_length + HEADER_LENGTH + FOOTER_LENGTH):]

                                else:
                                    #print("sum not OK") # デバッグ用
                                    # SUMが違ったので先頭バイトを削除
                                    receive_buffer = receive_buffer[1:]
                                    #print(receive_buffer) # デバッグ用

                            else:
                                #print("not ETX") # デバッグ用
                                # ETXが違ったので先頭バイトを削除
                                receive_buffer = receive_buffer[1:]

                        else:
                            #print("not CR") # デバッグ用
                            # CRが違ったので先頭バイトを削除
                            receive_buffer = receive_buffer[1:]

                    else:
                        #バッファ長が (データ長+ヘッダ長+フッタ長)未満なら
                        continue

                else:
                    #バッファ長がヘッダー長(4バイト)未満なら
                    continue

            else:
                #print("STX error")
                # STXがデータの先頭に無かったので先頭バイトを削除
                receive_buffer = receive_buffer[1:]





# インベントリのレスポンスデータ(コマンドが0x6C)から、PC_UIIデータと、RSSI値を切り出す
def handle_inventory_response(data_frame, pc_uii_list, rssi_list):
    """
    インベントリのレスポンスデータを処理する。
    :param data_frame:  受信したデータフレーム
    :param pc_uii_list: PC+UIIを格納するリスト
    :param rssi_list:   RSSI値を格納するリスト
    :param return:      リストを直接変更しているので何も返さない
    """
    # Todo: data_frameの長さが、
    #       n + pc_uii_lengthよりも大きいことを確認する？

    # [インベントリ ACKレスポンス フォーマット]
    #  STX      0x02
    #  アドレス  0x00 #固定では無い
    #  コマンド  0x6C
    #  データ長  5 + n
    #  データ部  0x09   1バイト 詳細コマンド
    #           RSSI   2バイト
    #           RSSI
    #           ANGLE  1バイト
    #           n      1バイト　PC+UIIのバイト数
    #           PC+UII nバイト
    #  ETX      0x03
    #  SUM      SUM
    #  CR       0x0D

    # 9バイト目の 'n' をPC+UIIのバイト数に入力
    pc_uii_length = data_frame[8]
    # pc_uii_lengthの長さだけデータをスライス（切り出し）します
    pc_uii_data = data_frame[9:9 + pc_uii_length]

    # 切り出したデータをリストへ追加
    pc_uii_list.append(pc_uii_data)

    # RSSI値の計算
    rssi_value = convert_rssi(data_frame[5:7].hex())
    # RSSI値をリストへ追加
    rssi_list.append(rssi_value)


# インベントリ時のACKのレスポンスデータから読み取り枚数を取得する。
def check_inventory_ack_response(data_frame):
    """
    インベントリ時のACKのレスポンスデータから読み取り枚数を取得する。
    :param data_frame: 受信したデータフレーム
    :return: 読み取り枚数
    """
    #print(data_frame.hex()) #デバッグ用
    # 7バイト目と8バイト目（Pythonのインデックスでは6:8）が読み取り枚数
    # リトルエンディアンの順序で整数に変換
    read_count = int.from_bytes(data_frame[6:8], byteorder='little')
    #print(read_count) #デバッグ用
    return read_count


# データフレームを解析し、STX-ETX-SUM-CRまでを抜き出す
def parse_data_frame(data, index):
    """
    受信したデータの構造を解析して各要素を意味のある単位に分割または抽出する
    :param data: 受信したデータ
    :param index: 現在の解析開始位置
    :return: 解析したデータフレームと次の開始位置
    """
    if len(data) >= (index + HEADER_LENGTH + FOOTER_LENGTH):
        # 4バイト目のデータ長を確認し、データ全体の長さを算出
        data_length = data[index + 3] + HEADER_LENGTH + FOOTER_LENGTH
        # データが最後まであるかを確認
        if len(data) >= (index + data_length):
            # データの最後がCR(0x0D)であれば
            if data[(index + data_length) - 1] == CR[0]:
                # 解析したデータフレームと次の開始位置を戻す
                return data[index:(index + data_length)], (index + data_length)

    # データが短ければ、元の開始位置のみを返す
    return None, index


# シリアル受信したデータ(受信解析後)の中からタグの情報などを抜き出す
# インベントリコマンド用
def received_data_parse(data):
    """
    受信データを解析する。
    :param data: 受信データ
    :return: 解析結果としてのPC+UIIリスト、RSSIリスト、期待される読み取り枚数
    """
    #print(data.hex()) #デバッグ用
    pc_uii_list: List[str] = []  # UII格納用    / 型ヒント:文字列
    rssi_list: List[float] = []  # RSSI値格納用 / 型ヒント:浮動小数点数
    expected_read_count = None   # ACKレスポンスから取得した読み取り枚数

    i = 0
    while i < len(data):
        if data[i] == STX[0]:   # STX: 0x02 があれば
            #print("STXあり")   # デバッグ用
            # データ解析
            data_frame, next_idx = parse_data_frame(data, i)

            if data_frame:
                if verify_sum_value(data_frame):
                    # 念のためSUM値の確認--すでに受信時に確認はしているが。。。
                    # フレームの3バイト目(CMD_LOCATION)をcommandにいれる
                    command = bytes([data_frame[CMD_LOCATION]])
                    detail_command = bytes([data_frame[DETAIL_LOCATION]])

                    if command == INV:
                        # コマンドに0x6Cがあれば、インベントリの応答として扱う
                        # 複数の受信コマンドに対応する場合は、要検討
                        handle_inventory_response(data_frame, pc_uii_list, rssi_list)

                    elif command == ACK:
                        if detail_command == DETAIL_INV:
                            # コマンドにACK:0x30があれば、ACKの応答として扱う
                            expected_read_count = check_inventory_ack_response(data_frame)
                        else:
                            continue

                    elif command == NACK:
                        # コマンドにNACK:0x31があれば、NACKの応答として扱う
                        print(parse_nack_response(data_frame))

                else:
                    print("サム値が正しくありません")
                    # この処理以前に解析したデータ(タグIDなど）を返す
                    return pc_uii_list, rssi_list, expected_read_count

                # 次の開始位置に変更
                i = next_idx

            else:
                print("データがありません")
                # この処理以前に解析したデータ(タグIDなど）を返す
                return pc_uii_list, rssi_list, expected_read_count
        else:
            i += 1

    if expected_read_count is not None:
        if expected_read_count != len(pc_uii_list):
            # 以下、(受信経路や上位のノイズなどで)受信データの不整合が
            # 発生したときに表示されます。(デバッグコードではありません)
            print("タグの読み取り数とpc_uii_listの個数が一致しません")
            print("タグの読み取り予定数: ", expected_read_count)
            print("pc_uii_listの個数 : ",  len(pc_uii_list))

    # 解析したデータ(タグIDなど）を返す
    return pc_uii_list, rssi_list, expected_read_count



# NACK応答時のエラー解析(初歩、一例) 全部は網羅してません。
def parse_nack_response(nack_response):

    if len(nack_response) < (HEADER_LENGTH + FOOTER_LENGTH):
        return "Invalid NACK response"

    # エラーコードを取得
    error_code = nack_response[5]

    error_messages = {
        0x01: "CMD_CRC_ERROR: データのCRCが一致しない",
        0x02: "CMD_TIME_OVER: データが途中で途切れた",
        0x03: "CMD_RX_ERROR: アンチコリジョン処理中にエラー",
        0x04: "CMD_RXBUSY_ERROR: RFタグからの応答がない",
        0x07: "CMD_ERROR: コマンド実行中にリーダライタ内部でエラー",
        0x0A: "CMD_UHF_IC_ERROR: RFタグアクセス時の内蔵チップエラー",
        0x60: "CMD_LBT_ERROR: キャリアセンス時のタイムアウトエラー",
        0x64: "HARDWARE_ERROR: ハードウェア内部で異常が発生",
        0x68: "CMD_ANT_ERROR: アンテナ断線検知エラー",
        0x42: "SUM_ERROR: 上位機器から送信されたコマンドのSUM値が正しくない",
        0x44: "FORMAT_ERROR: 上位機器から送信されたコマンドのフォーマットまたはパラメータが正しくない",
    }

    return error_messages.get(error_code, "Unknown NACK error")


# SUM値計算
def calculate_sum_value(data):
    """
    サム値を計算する。
    :param data: サム値を計算するデータのバイト列(STX-ETXまで)
    :return: 計算されたサム値
    """
    sum_value = 0
    for byte in data:
        sum_value += byte
    sum_value &= 0xFF  # 下位1バイトに制限
    return sum_value


# サム値を検証する。
def verify_sum_value(data_frame):
    """
    サム値を検証する。
    STXからETXまでの合計値と、データ内にあるSUM値が
    一致するかを確認している
    :param data_frame: 検証するデータフレーム
    :return:           サム値が正しいかどうかのブール値
    """
    data_length = len(data_frame)

    expect_sum_value = data_frame[data_length - 2]

    calculated_sum_value = calculate_sum_value(data_frame[:data_length - 2])

    if  expect_sum_value == calculated_sum_value:
        return True
    else:
        return False


# RSSI値計算
def convert_rssi(rssi_hex_value):
    """
    RSSI値(無線信号強度の指標)を計算する。
    -- RSSI 値の算出方法 --
    レスポンスの6~7[byte]目を符号付き16ビットとして扱い、
    10進数に変換してから10で割ります。
    ※詳しくは、プロトコル仕様書を参照
        ---符号付き16進数整数と10進数の変換--

    :param rssi_hex_value: RSSIの16進数値
    :return:               変換されたRSSI値
    """
    # [2進数に変換]
    # 16進数から整数への変換         int(rssi_hex_value, 16)
    # 整数から2進数の文字列への変換   bin(int(rssi_hex_value, 16))
    # '0b' の削除                   bin(int(rssi_hex_value, 16))[2:]
    binary_value = bin(int(rssi_hex_value, 16))[2:]
    # [ビット反転]
    # リスト内包表記 ('1' if bit == '0' else '0' for bit in binary_value)
    # binary_value 文字列の各ビット（文字）に対して処理
    # 条件 bit == '0' が True の場合、'1' を返し、False の場合、'0' を返す
    # 文字列の結合   ''.join(...)
    inverted_binary_value = ''.join('1' if bit == '0' else '0' for bit in binary_value)
    # [1を足す, '0b' の削除]
    # 2進数から整数への変換          int(inverted_binary_value, 2)
    # 整数値の加算                  int(inverted_binary_value, 2) + 1
    # 整数から2進数の文字列への変換  bin(int(inverted_binary_value, 2) + 1)
    # '0b' の削除                  bin(int(inverted_binary_value, 2) + 1)[2:]
    added_binary_value = bin(int(inverted_binary_value, 2) + 1)[2:]
    # [10進数に変換]
    rssi_value = int(added_binary_value, 2)
    # マイナスをつけ、10で割った値を戻す
    return -rssi_value / 10


# ブザー制御コマンドを送信する関数
def send_buzzer_command(ser, response_type, sound_type):
    """
    ブザー制御コマンドを送信する関数
    :param ser:           シリアル通信情報を取得
    :param response_type: 応答要求（0x00: 応答を要求しない, 0x01: 応答を要求する）
        #今回のプログラムでは、0x01のみ使用ください。（応答が必須）
    :param sound_type:    ブザー音の種類（0x00: ピー, 0x01: ピッピッピ, ..., 0x08: ピッピッピッピッ）
    """

    # データ部分の結合
    # respomse_type : 応答あり、なし
    # sound_type    : ブザー音の種類
    data = bytes([response_type, sound_type])

    # コマンドのヘッダ部分
    buzzer_command_header = STX + ADD + BUZ + bytes([len(data)])

    # STX-ETXまで結合
    full_command = buzzer_command_header + data + ETX

    # SUM値の計算
    sum_value = calculate_sum_value(full_command)  # STXとETXも含める

    # SUMとCRを追加
    full_command += bytes([sum_value]) + CR

    # コマンドを送信
    #print(full_command.hex()) # デバッグ用
    result = communicate(ser, full_command)
    #print(result.hex()) # デバッグ用
    return result


# 集計結果保存(参考)
def save_results_to_file(filename, total_iterations, total_read_time, total_read_count, pc_uii_count_dict):
    with open(filename, 'a', encoding="utf-8") as file:  # 'a' は追記モードでファイルを開く
        # 現在の日時を取得
        current_datetime = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        file.write("\n# -*- coding: utf-8 -*-\n")
        file.write(f"\n=== 集計結果 ({current_datetime}) ===\n")
        file.write(f"総繰り返し回数: {total_iterations}\n")
        file.write(f"総読み取り時間: {total_read_time:.2f} 秒\n")

        if total_iterations > 0:
            file.write(f"平均読み取り枚数: {total_read_count / total_iterations:.2f} 枚\n")

        file.write("各PC+UIIデータの読み取り回数:\n")

        for pc_uii_hex, count in pc_uii_count_dict.items():
            file.write(f"{pc_uii_hex}: {count} 回\n")

        file.write("========= ここまで ============\n\n\n")



# ここからメイン処理
def main():

    # 通信速度(ボーレート)の定義
    # UTRシリーズでは、115200bps
    BAUD_RATES = [115200, 57600, 38400, 19200, 9600]

    # シリアルポートのリストを取得
    devices = [info.device for info in list_ports.comports()]

    #シリアルコマンドの初期化
    ser: Optional[serial.Serial] = None

    # 利用可能デバイスを表示
    if not devices:  # デバイスが空（存在しない）場合
        print("エラー: 利用可能なデバイスがありません。")
        print("エラー: 接続を確認してください。")
        sys.exit(1)  # プログラムを終了

    else:
        print("利用可能デバイスの表示：")
        for idx, device in enumerate(devices):
            print(f"{idx}: {device}")

        print("複数ある場合、適切なデバイスを選択してください。")

    # デバイスを選択させる
    while True:
        user_input = input("どのデバイスを使いますか？[半角数字で]")
        if user_input.strip():  # 空白を削除した文字列が空でない場合
            try:
                device_number = int(user_input)
                if (device_number >= 0) and (device_number < len(devices)):
                    com_port = devices[device_number]
                    break  # 正しい入力値が得られたらループを抜けます
                else:
                    print("デバイス番号が範囲外です。正しい番号を入力してください。")
            except ValueError:
                print("エラー: 半角数字を入力してください。")
        else:
            print("エラー: 空白または空の文字列は無効です。")

    print(f"{com_port}に設定しました。")

    # UTRシリーズのボーレートは、通常 115200bpsなので、以下の処理は不要
    '''
    # ボーレートの表示
    print("ボーレートはいくつにしますか？")
    print("UTR-S201シリーズの標準は 115200 bps")
    for idx, rate in enumerate(BAUD_RATES):
        print(f"{idx}: {rate} bps")

    #ボーレートの選択
    while True:
        user_input = input()
        if user_input.strip():
            try:
                baud_rate_idx = int(user_input)
                if baud_rate_idx >=0 and baud_rate_idx < len(BAUD_RATES):
                    baud_rate = BAUD_RATES[baud_rate_idx]
                    break
                else:
                    print("正しい番号を入力してください。")
            except ValueError:
                print("エラー: 半角数字を入力してください。")
        else:
            print("エラー: 空白または空の文字列は無効です。")

    print(f"{baud_rate / 1000} kbpsに設定しました。")
    '''

    # UTRシリーズのボーレート(0: 115200bps)
    baud_rate = BAUD_RATES[0]

    # シリアルポートの設定
    try:
        # タイムアウト 1秒に設定 (最大1秒間はデータを待つ)
        ser = serial.Serial(com_port, baud_rate, timeout=1)

    except serial.SerialException as e:
        if "アクセスが拒否されました" in str(e):
            print(f"エラー: シリアルポート {com_port} は他のソフトウェアによって使用中です。")
        else:
            print(f"エラー: {e}")
        sys.exit(1)


    # ROMバージョン読み取りコマンドで通信確認
    result = communicate(ser, COMMANDS['ROM_VERSION_CHECK'])
    if re.match(STX + b'.' + ACK, result):  # b'.' は任意の1バイトを意味する:
    # ACKコマンドが帰ってきたら
    # 詳細コマンドを確認する
        if bytes([result[DETAIL_LOCATION]]) == DETAIL_ROM:
            print("シリアル通信: OK")
            #print(result.hex()) # デバッグ用

    elif re.match(STX + b'.' + NACK, result):
        # NACKだったら
        if bytes([result[DETAIL_LOCATION]]) == DETAIL_ROM:
            print(parse_nack_response(result))

    else:
        print("シリアル通信: NG")
        ser.close()
        sys.exit(1)

    # コマンドモードへ切り替え
    result = communicate(ser, COMMANDS['COMMAND_MODE_SET'])
    if re.match(STX + b'.' + ACK, result):  # b'.' は任意の1バイトを意味する:
        # ACKコマンドが帰ってきたら
        print("コマンドモードに切り替えました")

    elif re.match(STX + b'.' + NACK, result):
        # NACKだったら
        print(parse_nack_response(result))

    else:
        print("コマンドモード切り替えできませんでした")
        ser.close()
        sys.exit(1)


    # UHF送信出力設定読み取りコマンド(コマンドモード用)
    output_power_level = 0.0 # 送信出力初期化
    result = communicate(ser, COMMANDS['UHF_READ_OUTPUT_POWER'])
    #print(result)       # デバッグ用
    #print(result.hex()) # デバッグ用
    if re.match(STX + b'.' + ACK, result):  # b'.' は任意の1バイトを意味する:
        # ACKコマンドが帰ってきたら
        level_hex = hex(result[8]+result[7])
        #print(level_hex) # デバッグ用
        output_power_level = int(level_hex, 16) / 10
        print("送信出力値：", output_power_level, "dBm")

    elif re.match(STX + b'.' + NACK, result):
        # NACKだったら
        print(parse_nack_response(result))

    else:
        print("通信エラー")
        print(result.hex())
        ser.close()
        sys.exit(1)


    # UHF送信周波数チャンネル読み取りコマンド(コマンドモード用)
    output_ch = 0 # チャンネル番号初期化
    result = communicate(ser, COMMANDS['UHF_READ_FREQ_CH'])
    #print(result.hex()) # デバッグ用
    if re.match(STX + b'.' + ACK, result):  # b'.' は任意の1バイトを意味する:
        # ACKコマンドが帰ってきたら
        # 8バイト目のチャンネル情報を取得
        output_ch_hex = hex(result[7])
        #print(output_ch_hex)
        # HEX値をint型に変更
        output_ch = int(output_ch_hex, 16)
        print("チャンネル番号：", output_ch, "ch")
        print("送信周波数：", OUTPUT_CH_FREQ_LIST[output_ch-1], " MHz")

    elif re.match(STX + b'.' + NACK, result):
        # NACKだったら
        print(parse_nack_response(result))

    else:
        print("通信エラー")
        print(result.hex())
        ser.close()
        sys.exit(1)


    # UHF_GET_INVENTORY_PARAM
    result = communicate(ser, COMMANDS['UHF_GET_INVENTORY_PARAM'])
    if re.match(STX + b'.' + ACK, result):  # b'.' は任意の1バイトを意味する:
        # ACKコマンドが帰ってきたら
        print("UHF_GET_INVENTORY_PARAMが正常に実行されました")

    elif re.match(STX + b'.' + NACK, result):  # b'.' は任意の1バイトを意味する:
        # NACKだったら
        print(parse_nack_response(result))

    else:
        print("UHF_GET_INVENTORY_PARAMが正常に実行できませんでした")
        print(result.hex())
        ser.close()
        sys.exit(1)


    # UHF_SET_INVENTORY_PARAM
    result = communicate(ser, COMMANDS['UHF_SET_INVENTORY_PARAM'])
    if re.match(STX + b'.' + ACK, result):  # b'.' は任意の1バイトを意味する:
        # ACKコマンドが帰ってきたら
        print("UHF_SET_INVENTORY_PARAMが正常に実行されました")

    elif re.match(STX + b'.' + NACK, result):  # b'.' は任意の1バイトを意味する:
        # NACKだったら
        print(parse_nack_response(result))

    else:
        print("UHF_SET_INVENTORY_PARAMが正常に実行できませんでした")
        print(result.hex())
        ser.close()
        sys.exit(1)


    # 集計用の変数
    total_read_time   = 0.0  # 読み取りに要した総時間
    total_read_count  = 0    # 総読み取り枚数
    total_iterations  = 0    # 総繰り返し回数
    pc_uii_count_dict = {}   # 各PC+UIIデータの読み取り回数


    # 読み取り回数の指定(1-100)
    # 整数以外または100より大きい場合にエラー処理
    while True:
        try:
            print(" ")
            repeat_count = int(input("繰り返す回数を入力してください: "))
            if (repeat_count <= 0) or (repeat_count > 100):
                raise ValueError("入力は1から100の整数である必要があります。")
            break  # 正しい入力値が得られたらループを抜けます
        except ValueError as e:
            print(f"エラー: {e}")

    # 指定回数だけ繰り返し
    for _ in range(repeat_count):
        # タイムスタンプ（開始時間）を取得
        start_time = time.time()

        # インベントリコマンド送信
        received_data_bytes = communicate(ser, COMMANDS['UHF_INVENTORY'])
        #print(received_data_bytes.hex()) # デバッグ用

        # データを解析（インベントリ用）
        pc_uii_data_list, rssi_list, expected_read_count = received_data_parse(received_data_bytes)

        # タイムスタンプ（終了時間）を取得し、読み取り時間を計算
        end_time = time.time()
        read_time = end_time - start_time
        #print(read_time) # デバッグ用

        # 集計
        total_read_time += read_time # 読み取り時間を加算
        total_iterations += 1        # 繰り返し回数を加算
        if expected_read_count is not None:
            total_read_count += expected_read_count
            # 総読み取り枚数へ加算

        for pc_uii_data in pc_uii_data_list:
            pc_uii_hex = pc_uii_data.hex()
            # 16進数に変換して格納
            if pc_uii_hex not in pc_uii_count_dict:
                pc_uii_count_dict[pc_uii_hex] = 1   # 新規ならUII登録しカウント値1をセット
            else:
                pc_uii_count_dict[pc_uii_hex] += 1  # 新規でなければカウント値を1インクリメント

        #print(pc_uii_count_dict) # デバッグ用

        if pc_uii_data_list != []:
            # ブザー タグを読み取っていたら
            # 応答を要求する (0x01) 、ブザー音は「ピー」 (0x00)
            result = send_buzzer_command(ser, 0x01, 0x00)
            # ブザーコマンドでNACKになることは稀 (以下の動作は、未確認)
            if re.match(STX + b'.' + NACK, result):  # b'.' は任意の1バイトを意味する
                # NACKコマンドが帰ってきたら
                print("ブザーパラメータが間違っています")


            print("タグを " + str(expected_read_count) + " 枚読み取りました。")

        else:
            # ブザー タグを読み取っていない場合
            # 応答を要求する (0x01) 、ブザー音は「ピッピッピ」 (0x01)
            result = send_buzzer_command(ser, 0x01, 0x01)
            # ブザーコマンドでNACKになることは稀 (以下の動作は、未確認)
            if re.match(STX + b'.' + NACK, result):  # b'.' は任意の1バイトを意味する
                # NACKコマンドが帰ってきたら
                print("ブザーパラメータが間違っています")


            print("タグが見つかりませんでした")

        # pc_uii_data_list と rssi_list という2つのリストを zip() で結合
        # pc_uii_data_list の1つ目の要素と rssi_list の1つ目の要素が、
        # 1回目のループで pc_uii_data と rssi_value にそれぞれ代入されます。
        # pc_uii_data_list の2つ目の要素と rssi_list の2つ目の要素が、
        # 2回目のループで代入されます。
        # これを両リストの要素がなくなるまで繰り返し
        for pc_uii_data, rssi_value in zip(pc_uii_data_list, rssi_list):
            print("RSSI値:", rssi_value, "/ PC+UIIデータ:", pc_uii_data.hex())



    # 結果の出力
    print("\n=== 集計結果 ===")
    print(f"総繰り返し回数: {total_iterations}")
    print(f"総読み取り時間: {total_read_time:.2f} 秒")
    if total_iterations > 0:
        print(f"平均読み取り枚数: {total_read_count / total_iterations:.2f} 枚")

    print("各PC+UIIデータの読み取り回数:")

    for pc_uii_hex, count in pc_uii_count_dict.items():
        print(f"{pc_uii_hex}: {count} 回")



    # 集計結果保存
    filename = "Inventory_result.log"
    save_results_to_file(filename, total_iterations, total_read_time, total_read_count, pc_uii_count_dict)


    # シリアル通信を閉じる
    ser.close()

# スクリプトが直接実行されたときのみ main() を呼び出す。
if __name__ == "__main__":
    main()
