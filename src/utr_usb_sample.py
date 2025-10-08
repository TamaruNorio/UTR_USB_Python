
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UTR-S201 シリーズ（USBシリアル接続）サンプルプログラム（無保証） V1.1.5
Python 3.10+ / Windows 10+ で動作確認想定

【概要】
このプログラムは、タカヤ製UTR-S201シリーズRFIDリーダライタ（USBシリアル接続モデル）を
Pythonから操作するためのGUIを持たないコマンドラインサンプルです。

【注意事項】
- すべての条件分岐を網羅（確認）しているわけではありません。
- 作りこみには、当社の製品の基礎的な知識が必要です。
- 該当製品のプロトコル仕様書も理解する必要があります。
- 機器の設定には、別途RWManagerを使用し、実際の動作における通信ログなどを確認ください。
- タイムアウトや再送、パケット分割等は環境に合わせて調整してください。

【前提】
- 事前にpyserialのインストールが必要です。
- Raspberry Pi4 (Raspbian GNU/Linux 11 (bullseye)) および Windows 10+ で動作確認されています。

【更新履歴】
- Revised :  `communicate(command, timeout=1)` 関数変更
             受信データ内部のSTX,ETX,SUM,CRのチェックを実施
             タイムアウト or ACK/NACK受信 にて、ループを抜ける
- 集計結果をファイル保存（追加モード）
- `ser`をglobalからローカルへ変更、引数に設定。
- 詳細コマンド確認を追加(ROMバージョンチェックのみ)
- 変数（小文字）と定数（大文字）に変更
- 定数名を変更（CMD_LOCATION、DETAIL_LOCATION、OUTPUT_CH_FREQ_LIST）

【TODOリスト】
- ACK/NACKの判断をする別関数があっても良いかもしれません。
- 文字入力も別関数でチェック（処理）しても良いかもしれません。
- 送信出力の変更ができれば良いかもしれません。
- 連続インベントリモードへの対応（受信のみ。パケット途中からとか）
  現状の関数だとACK/NACK受信しないと戻らない（コマンドモードのみ対応）
- アンテナ選択、設定変更などをどうするか検討が必要です。
- OUTPUT_CH_FREQ_LISTを辞書型にするか検討が必要です。
- レスポンスデータの詳細コマンドまで確認するようにすべきです。
- 経過時間の取得方法の検討（差分がマイナスになり正しく時間計測できない場合の検討）

【参考情報】
R/W(リーダライタ)の各種動作設定については、Windows版 UTR-RWManagerを使用してください。
シリアル通信コマンドの詳細は、通信プロトコル仕様書を熟読のこと。
[https://www.product.takaya.co.jp/rfid/download/uhf.html/](https://www.product.takaya.co.jp/rfid/download/uhf.html/)

[コマンド、レスポンスの基本フォーマット]
- STX(02h) 1バイト #Start Text
- アドレス  1バイト (RWのIDなど、デフォルトは 00h)
- コマンド  1バイト
- データ長  1バイト
- データ部  0～255バイト
- ETX(03h) 1バイト  #End Text
- SUM      1バイト　STXからETXまで
- CR(0Dh)  1バイト  '/r' キャリッジリターン
"""

# 関連モジュールをインポート
import sys
import time
import datetime
import re

import serial
from   serial.tools import list_ports

from   typing       import List, Optional, Tuple


# UTR用 シリアル送信コマンドの定義
# 各コマンドはバイト列として定義されており、そのまま送信可能
COMMANDS = {
    # ROMバージョンの読み取りコマンド: リーダライタのファームウェアバージョンを確認
    'ROM_VERSION_CHECK': bytes([0x02, 0x00, 0x4F, 0x01, 0x90, 0x03, 0xE5, 0x0D]),
    # コマンドモードへの切り替えコマンド: リーダライタをコマンド制御可能な状態に設定
    'COMMAND_MODE_SET': bytes([0x02, 0x00, 0x4E, 0x07, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x03, 0x6A, 0x0D]),
    # UHF_Inventoryコマンド: RFタグを読み取るためのインベントリ操作を開始
    'UHF_INVENTORY': bytes([0x02, 0x00, 0x55, 0x01, 0x10, 0x03, 0x6B, 0x0D]),
    # UHF_GetInventoryParam: RFタグ読み取り時のインベントリ処理に使用するパラメータの取得
    'UHF_GET_INVENTORY_PARAM':bytes([0x02,0x00,0x55,0x02,0x41,0x00,0x03,0x9D,0x0D]),
    # UHF_SetInventoryParamコマンド: インベントリパラメータの設定 (必要に応じて変更、別途関数あり)
    'UHF_SET_INVENTORY_PARAM': bytes([0x02,0x00,0x55,0x09,0x30,0x00,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x14,0x0D]),
    # UHF送信出力設定読み取りコマンド: リーダライタの送信出力レベルを読み取り
    'UHF_READ_OUTPUT_POWER': bytes([0x02,0x00,0x55,0x03,0x43,0x01,0x00,0x03,0xA1,0x0D]),
    # UHF送信周波数チャンネル読み取りコマンド: リーダライタの送信周波数チャンネルを読み取り
    'UHF_READ_FREQ_CH': bytes([0x02,0x00,0x55,0x03,0x43,0x02,0x00,0x03,0xA2,0x0D]),
    # UHFブザー制御コマンド:応答要求＋ピー音
    'UHF_BUZZER_pi': bytes([0x02,0x00,0x42,0x02,0x01,0x00,0x03,0x4A,0x0D]),
    # UHFブザー制御コマンド:応答要求＋ピッピッピ音
    'UHF_BUZZER_pipipi': bytes([0x02,0x00,0x42,0x02,0x01,0x01,0x03,0x4B,0x0D]),
    # UHF書き込みコマンド（一例、使ってません）: RFタグへのデータ書き込み (このサンプルでは未使用)
    'UHF_WRITE': bytes([0x02,0x00,0x55,0x08,0x16,0x01,0x00,0x00,0x00,0x02,0x04,0x56,0x03,0xD5,0x0D]),
}

# 定数の定義 (プロトコル仕様に準拠)
HEADER_LENGTH     = 4        # STX, アドレス, コマンド, データ長 (各1バイト) の合計長
FOOTER_LENGTH     = 3        # ETX, SUM, CR (各1バイト) の合計長
STX : bytes       = b'\x02'  # Start Text: フレーム開始を示すバイト
ADD : bytes       = b'\x00'  # アドレス: リーダライタIDなど (設定により変化することあり)
ETX : bytes       = b'\x03'  # End Text: データ部の終了を示すバイト
CR  : bytes       = b'\x0D'  # Carriage Return: フレームの終端を示すバイト
ACK : bytes       = b'\x30'  # ACK (Acknowledgment): 正常応答
NACK: bytes       = b'\x31'  # NACK (Negative Acknowledgment): エラー応答
INV : bytes       = b'\x6C'  # インベントリコマンド: RFタグ読み取りコマンド
BUZ : bytes       = b'\x42'  # ブザーコマンド: ブザー制御コマンド
CMD_LOCATION      = 2        # フレーム内のコマンドバイトの位置 (0-indexed)
DETAIL_LOCATION   = 4        # フレーム内の詳細コマンドバイトの位置 (0-indexed)
DETAIL_ROM: bytes = b'\x90'  # ROMバージョン読み取りの詳細コマンド
DETAIL_INV: bytes = b'\x10'  # インベントリの詳細コマンド

# 出力チャンネルと周波数のマッピングリスト (MHz)
OUTPUT_CH_FREQ_LIST = [916.0, 916.2, 916.4, 916.6, 916.8, 917.0, 917.2, 917.4, 917.6, 917.8, 918.0, 918.2, 918.4, 918.6, 918.8, 919.0, 919.2, 919.4, 919.6, 919.8, 920.0, 920.2, 920.4, 920.6, 920.8, 921.0, 921.2, 921.4, 921.6, 921.8, 922.0, 922.2, 922.4, 922.6, 922.8, 923.0, 923.2, 923.4]

#【シリアルデータ通信関数】
# データ送信後に、受信データ解析を実施
# 制御フロー参照: https://www.product.takaya.co.jp/dcms_media/other/TDR-OTH-PROGRAMMING-103.pdf
# タイムアウトもしくは、ACK, NACKを受信したらループを抜ける
# 受信データが多い(たくさんのタグ:30枚以上を読み取りする場合など)は、
# タイムアウト時間を多くする必要あり。もしくはタイムアウト処理の方法を変更する。
def communicate(ser: serial.Serial, command: bytes, timeout: float = 1.0) -> bytes:
    """
    コマンドをシリアルポートに送信し、応答を受信して解析する。
    受信データを1バイトずつ解析(STX, CR, ETX, SUM)しながら
    正常なレスポンスコマンドのみを連結して返す。
    ACK/NACKを受信するか、タイムアウトが発生したら処理を終了する。

    Args:
        ser (serial.Serial): シリアル通信オブジェクト。
        command (bytes): 送信するコマンドバイト列。
        timeout (float): 受信タイムアウト時間（秒）。デフォルトは1秒。

    Returns:
        bytes: 受信した完全な応答フレームのバイト列。
    """
    complete_response  = b'' # 解析後の正常レスポンスを格納するバッファ
    receive_buffer: List[int] = []      # 受信バイトを一時的に保持するリスト
    buffer_length = 0        # receive_bufferの現在の長さ
    data_length = 0          # フレーム内のデータ長

    if ser is not None:
        # コマンド送信 (上位 -> RW)
        ser.write(command)

    # タイム計測開始（現在の時刻を取得）
    start_time = time.time()

    # シリアル受信、データ解析処理
    while True:
        # タイムアウト処理
        if (time.time() - start_time) > timeout:
            print("タイムアウト: レスポンスが一定時間内に受信されませんでした。")
            return complete_response

        if ser is not None:
            # 受信バッファにデータがあれば、1バイトずつ読み取り
            # ser.read(1)はバイト列を返すため、リストに格納するためにintに変換
            chunk = ser.read(1)
            if chunk:
                receive_buffer.append(chunk[0])
                buffer_length = len(receive_buffer) # バッファーの長さを取得

        # receive_buffer内になにかデータがあり、先頭がSTX(0x02)だったら、
        if receive_buffer:
            if receive_buffer[0] == STX[0]:
                # バッファ長がヘッダー長(4バイト)以上であれば、
                if buffer_length >= HEADER_LENGTH:
                    # データ長(4バイト目)を data_lengthに入力
                    data_length = receive_buffer[HEADER_LENGTH - 1]

                    # バッファ長が (データ長+ヘッダ長+フッタ長)以上であれば、
                    if buffer_length >= (data_length + HEADER_LENGTH + FOOTER_LENGTH):
                        # 最後位がCR(0x0D)であれば、
                        if receive_buffer[(data_length + HEADER_LENGTH + FOOTER_LENGTH) - 1] == CR[0]:
                            # ETXの位置にETX(0x03)があれば、
                            if receive_buffer[data_length + HEADER_LENGTH] == ETX[0]:
                                # SUM値の確認がTrueであれば、
                                # バイト列に変換してverify_sum_valueに渡す
                                current_frame_bytes = bytes(receive_buffer[:(data_length + HEADER_LENGTH + FOOTER_LENGTH)])
                                if verify_sum_value(current_frame_bytes):
                                    # 戻り値に、フォーマット確認済みのバッファを追加
                                    complete_response += current_frame_bytes

                                    # ACK, NACK受信していたら抜ける
                                    if receive_buffer[CMD_LOCATION] in [ACK[0], NACK[0]]:
                                        receive_buffer = [] # バッファをクリア
                                        return complete_response

                                    # 確認済みバッファをクリアし、次のフレームの解析へ
                                    receive_buffer = receive_buffer[(data_length + HEADER_LENGTH + FOOTER_LENGTH):]

                                else:
                                    # SUMが違ったので先頭バイトを削除し、再同期を試みる
                                    receive_buffer = receive_buffer[1:]

                            else:
                                # ETXが違ったので先頭バイトを削除し、再同期を試みる
                                receive_buffer = receive_buffer[1:]

                        else:
                            # CRが違ったので先頭バイトを削除し、再同期を試みる
                            receive_buffer = receive_buffer[1:]

                    else:
                        # バッファ長が (データ長+ヘッダ長+フッタ長)未満なら、まだフレームが完全でないので継続
                        continue

                else:
                    # バッファ長がヘッダー長(4バイト)未満なら、まだヘッダーが完全でないので継続
                    continue

            else:
                # STXがデータの先頭に無かったので先頭バイトを削除し、STXを探索
                receive_buffer = receive_buffer[1:]


# インベントリのレスポンスデータ(コマンドが0x6C)から、PC_UIIデータと、RSSI値を切り出す
def handle_inventory_response(data_frame: bytes, pc_uii_list: List[bytes], rssi_list: List[float]) -> None:
    """
    インベントリのレスポンス1フレームからPC+UIIデータとRSSI値を抽出し、リストに格納する。

    Args:
        data_frame (bytes): 受信したデータフレーム（STXからCRまで）。
        pc_uii_list (List[bytes]): 抽出したPC+UIIデータを格納するリスト。
        rssi_list (List[float]): 抽出したRSSI値を格納するリスト。
    """
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

    # RSSI値の計算 (フレームの5バイト目と6バイト目を使用)
    rssi_value = convert_rssi(data_frame[5:7].hex())
    # RSSI値をリストへ追加
    rssi_list.append(rssi_value)


# インベントリ時のACKのレスポンスデータから読み取り枚数を取得する。
def check_inventory_ack_response(data_frame: bytes) -> int:
    """
    インベントリ時のACK応答フレームから、期待される読み取り枚数を抽出して返す。

    Args:
        data_frame (bytes): 受信したACK応答フレーム。

    Returns:
        int: 期待される読み取り枚数。
    """
    # 7バイト目と8バイト目（Pythonのインデックスでは6:8）が読み取り枚数
    # リトルエンディアンの順序で整数に変換
    read_count = int.from_bytes(data_frame[6:8], byteorder='little')
    return read_count


# データフレームを解析し、STX-ETX-SUM-CRまでを抜き出す
def parse_data_frame(data: bytes, index: int) -> Tuple[Optional[bytes], int]:
    """
    受信したデータバイト列から、STXからCRまでの完全な1フレームを抽出する。

    Args:
        data (bytes): 受信したデータバイト列。
        index (int): 現在の解析開始位置。

    Returns:
        Tuple[Optional[bytes], int]: 
            - 抽出されたフレーム (bytes)。完全なフレームが見つからない場合はNone。
            - 次の解析開始位置 (int)。
    """
    # フレームの最小長をチェック (ヘッダー長 + フッター長)
    if len(data) >= (index + HEADER_LENGTH + FOOTER_LENGTH):
        # 4バイト目のデータ長を確認し、フレーム全体の長さを算出
        data_length_in_frame = data[index + 3] # フレーム内のデータ長フィールド
        total_frame_len = data_length_in_frame + HEADER_LENGTH + FOOTER_LENGTH

        # フレーム全体がバッファに存在するかを確認
        if len(data) >= (index + total_frame_len):
            # データの最後がCR(0x0D)であれば
            if data[(index + total_frame_len) - 1] == CR[0]:
                # 解析したデータフレームと次の開始位置を戻す
                return data[index:(index + total_frame_len)], (index + total_frame_len)

    # データが短かったり、完全なフレームが見つからない場合は、元の開始位置のみを返す
    return None, index


# シリアル受信したデータ(受信解析後)の中からタグの情報などを抜き出す
# インベントリコマンド用
def received_data_parse(data: bytes) -> Tuple[List[bytes], List[float], Optional[int]]:
    """
    受信したバイト列から複数のフレームを走査し、インベントリ結果を解析する。
    PC+UIIリスト、RSSIリスト、期待される読み取り枚数を抽出する。

    Args:
        data (bytes): 受信した生のデータバイト列。

    Returns:
        Tuple[List[bytes], List[float], Optional[int]]: 
            - pc_uii_list (List[bytes]): 読み取られたPC+UIIデータのリスト。
            - rssi_list (List[float]): 読み取られたRSSI値のリスト。
            - expected_read_count (Optional[int]): 期待される読み取りタグ数 (ACKフレームから取得)。
    """
    pc_uii_list: List[bytes] = []  # UII格納用
    rssi_list: List[float] = []  # RSSI値格納用
    expected_read_count: Optional[int] = None   # ACKレスポンスから取得した読み取り枚数

    i = 0
    while i < len(data):
        if data[i] == STX[0]:   # STX: 0x02 があればフレームの開始とみなす
            # データ解析
            data_frame, next_idx = parse_data_frame(data, i)

            if data_frame:
                if verify_sum_value(data_frame): # チェックサムを検証
                    # フレームの3バイト目(CMD_LOCATION)をcommandにいれる
                    command = bytes([data_frame[CMD_LOCATION]])
                    # 詳細コマンドを抽出 (存在する場合)
                    detail_command = bytes([data_frame[DETAIL_LOCATION]]) if len(data_frame) > DETAIL_LOCATION else b''

                    if command == INV:
                        # コマンドが0x6C (INV) なら、インベントリの応答として処理
                        handle_inventory_response(data_frame, pc_uii_list, rssi_list)

                    elif command == ACK:
                        # コマンドがACK (0x30) なら、ACKの応答として処理
                        if detail_command == DETAIL_INV:
                            # 詳細コマンドがインベントリACK (0x10) なら、読み取り枚数を取得
                            expected_read_count = check_inventory_ack_response(data_frame)
                        # else: 他のACK応答はここでは特に処理しない

                    elif command == NACK:
                        # コマンドがNACK (0x31) なら、NACKの応答として処理
                        print(parse_nack_response(data_frame)) # NACKエラーメッセージを表示

                else:
                    print("サム値が正しくありません（途中までの結果を返します）")
                    # この処理以前に解析したデータ(タグIDなど）を返す
                    return pc_uii_list, rssi_list, expected_read_count

                # 次の開始位置に変更
                i = next_idx

            else:
                # 完全なフレームが見つからなかった場合、現在の解析を打ち切る
                print("データがありません、または不完全なフレームです")
                # この処理以前に解析したデータ(タグIDなど）を返す
                return pc_uii_list, rssi_list, expected_read_count
        else:
            # STXが見つからない場合は1バイト進める
            i += 1

    # 期待される読み取り枚数と実際に読み取った枚数が一致しない場合の警告
    if expected_read_count is not None:
        if expected_read_count != len(pc_uii_list):
            # 以下、(受信経路や上位のノイズなどで)受信データの不整合が
            # 発生したときに表示されます。(デバッグコードではありません)
            print("タグの読み取り数とpc_uii_listの個数が一致しません")
            print("タグの読み取り予定数: ", expected_read_count)
            print("pc_uii_listの個数   : ",  len(pc_uii_list))

    # 解析したデータ(タグIDなど）を返す
    return pc_uii_list, rssi_list, expected_read_count


# NACK応答時のエラー解析(初歩、一例) 全部は網羅してません。
def parse_nack_response(nack_response: bytes) -> str:
    """
    NACK応答フレームのエラーコードを解析し、対応するエラーメッセージを返す。

    Args:
        nack_response (bytes): 受信したNACK応答フレーム。

    Returns:
        str: エラーメッセージ。
    """
    if len(nack_response) < (HEADER_LENGTH + FOOTER_LENGTH):
        return "Invalid NACK response" # NACK応答フレームが短すぎる場合

    # エラーコードを取得 (通常はフレームの6バイト目、インデックス5)
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

    return error_messages.get(error_code, f"Unknown NACK error (0x{error_code:02X})")


# SUM値計算
def calculate_sum_value(data: bytes) -> int:
    """
    バイト列の合計値（チェックサム）を計算する。
    STXからETXまでのバイトの合計の下位1バイトを返す。

    Args:
        data (bytes): チェックサムを計算するデータのバイト列 (STX-ETXまで)。

    Returns:
        int: 計算されたチェックサム（下位1バイト）。
    """
    sum_value = 0
    for byte in data:
        sum_value += byte
    sum_value &= 0xFF  # 下位1バイトに制限 (256で割った余り)
    return sum_value


# サム値を検証する。
def verify_sum_value(data_frame: bytes) -> bool:
    """
    データフレーム内のチェックサムが正しいか検証する。
    STXからETXまでの合計値と、データ内にあるSUM値が一致するかを確認する。

    Args:
        data_frame (bytes): 検証するデータフレーム（STXからCRまで）。

    Returns:
        bool: チェックサムが正しい場合はTrue、そうでない場合はFalse。
    """
    data_length = len(data_frame)

    # フレームの最後から2番目のバイトが期待されるSUM値
    expect_sum_value = data_frame[data_length - 2]

    # STXからETXまでのバイトの合計を計算
    calculated_sum_value = calculate_sum_value(data_frame[:data_length - 2])

    return expect_sum_value == calculated_sum_value


# RSSI値計算
def convert_rssi(rssi_hex_value: str) -> float:
    """
    RSSI値(無線信号強度の指標)を計算する。
    レスポンスの6〜7バイト目を符号付き16ビットとして扱い、
    10進数に変換してから10で割る。
    ※詳しくは、プロトコル仕様書を参照

    Args:
        rssi_hex_value (str): RSSIの16進数文字列（例: "FFC0"）。

    Returns:
        float: 変換されたRSSI値（dBm）。
    """
    # 16進数文字列を整数に変換
    rssi_int = int(rssi_hex_value, 16)

    # 16ビット符号付き整数として扱う
    if rssi_int & 0x8000:  # 最上位ビットが1の場合（負の数）
        rssi_int = -(0x10000 - rssi_int)

    # 10で割ってdBm値に変換
    return rssi_int / 10.0


# 集計ログ保存
def save_results_to_file(filename: str, total_iterations: int, total_read_time: float,
                         total_read_count: int, pc_uii_count_dict: dict) -> None:
    """
    インベントリの結果を集計し、指定されたファイルに追記する。

    Args:
        filename (str): 結果を保存するファイル名。
        total_iterations (int): 総繰り返し回数。
        total_read_time (float): 総読み取り時間（秒）。
        total_read_count (int): 総読み取りタグ数。
        pc_uii_count_dict (dict): PC+UIIごとの読み取り回数を格納した辞書。
    """
    with open(filename, 'a', encoding="utf-8") as f:
        current_datetime = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        f.write("\n# -*- coding: utf-8 -*-\n") # ファイルエンコーディング指定
        f.write(f"\n=== 集計結果 ({current_datetime}) ===\n")
        f.write(f"総繰り返し回数: {total_iterations}\n")
        f.write(f"総読み取り時間: {total_read_time:.2f} 秒\n")
        if total_iterations > 0:
            f.write(f"平均読み取り枚数: {total_read_count / total_iterations:.2f} 枚\n")
        f.write("各PC+UIIデータの読み取り回数:\n")
        for pc_uii_hex, count in pc_uii_count_dict.items():
            f.write(f"{pc_uii_hex}: {count} 回\n")
        f.write("========= ここまで ============\n\n\n")


# メイン処理
def main():
    """
    プログラムのエントリポイント。
    シリアルポートの選択、リーダライタとの通信、コマンド実行、結果表示、集計保存を行う。
    """
    # --- 接続情報の入力 ---
    print("UTR（USBモデル）に接続します。")

    # 利用可能なシリアルポートを列挙
    ports = list_ports.comports()
    if not ports:
        print("利用可能なCOMポートが見つかりませんでした。")
        sys.exit(1)

    print("利用可能なCOMポート:")
    for i, p in enumerate(ports):
        print(f"  [{i}]: {p.device} - {p.description}")

    selected_port_index = -1
    while not (0 <= selected_port_index < len(ports)):
        try:
            selected_port_index = int(input("接続するCOMポートの番号を入力してください: "))
        except ValueError:
            print("無効な入力です。数字を入力してください。")

    port_name = ports[selected_port_index].device

    baud_rate_str = input("ボーレートを入力してください（例: 19200, 115200, 未入力なら19200）: ").strip()
    baud_rate = int(baud_rate_str) if baud_rate_str else 19200

    # --- シリアルポート設定 ---
    ser: Optional[serial.Serial] = None
    try:
        # シリアルポートを開く (timeout=0 でノンブロッキング読み取り)
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            timeout=0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        ser.reset_input_buffer()  # 入力バッファをクリア
        ser.reset_output_buffer() # 出力バッファをクリア
        print(f"接続成功: {port_name} @ {baud_rate}bps")
    except serial.SerialException as e:
        print(f"シリアルポート接続エラー: {e}")
        sys.exit(1)

    # --- ROMバージョンで通信確認 ---
    # ROMバージョン確認コマンドを送信し、応答を待つ
    result = communicate(ser, COMMANDS['ROM_VERSION_CHECK'])
    # 応答がACKで、詳細コマンドがROMバージョン確認のものであるかチェック
    if re.match(STX + b'.' + ACK, result):
        if bytes([result[DETAIL_LOCATION]]) == DETAIL_ROM:
            print("USB通信: OK（ROMバージョン ACK 受信）")
    # 応答がNACKの場合
    elif re.match(STX + b'.' + NACK, result):
        if bytes([result[DETAIL_LOCATION]]) == DETAIL_ROM:
            print(parse_nack_response(result))
    # その他の応答の場合
    else:
        print("USB通信: NG（ACK/NACK なし）")
        ser.close()
        sys.exit(1)

    # --- コマンドモード切替 ---
    # コマンドモード設定コマンドを送信
    result = communicate(ser, COMMANDS['COMMAND_MODE_SET'])
    if re.match(STX + b'.' + ACK, result):
        print("コマンドモードに切り替えました")
    elif re.match(STX + b'.' + NACK, result):
        print(parse_nack_response(result))
    else:
        print("コマンドモード切替に失敗しました")
        ser.close()
        sys.exit(1)

    # --- 出力/周波数の読み取り ---
    # 出力電力の読み取り
    result = communicate(ser, COMMANDS['UHF_READ_OUTPUT_POWER'])
    if re.match(STX + b'.' + ACK, result):
        # 応答から出力レベルを抽出し、dBmに変換して表示
        # 8バイト目と7バイト目を結合して16進数として扱い、10進数に変換後10で割る
        level_hex = result[7:9].hex() # 7バイト目と8バイト目を抽出
        output_power_level = int(level_hex, 16) / 10.0
        print("送信出力値：", output_power_level, "dBm")
    elif re.match(STX + b'.' + NACK, result):
        print(parse_nack_response(result))
    else:
        print("通信エラー（UHF_READ_OUTPUT_POWER）")
        print(result.hex())
        ser.close()
        sys.exit(1)

    # 周波数チャンネルの読み取り
    result = communicate(ser, COMMANDS['UHF_READ_FREQ_CH'])
    if re.match(STX + b'.' + ACK, result):
        # 応答からチャンネル番号を抽出し、対応する周波数を表示
        output_ch = result[7] # 8バイト目がチャンネル番号
        print("チャンネル番号：", output_ch, "ch")
        if 1 <= output_ch <= len(OUTPUT_CH_FREQ_LIST):
            print("送信周波数：", OUTPUT_CH_FREQ_LIST[output_ch-1], " MHz")
    elif re.match(STX + b'.' + NACK, result):
        print(parse_nack_response(result))
    else:
        print("通信エラー（UHF_READ_FREQ_CH）")
        print(result.hex())
        ser.close()
        sys.exit(1)

    # --- インベントリパラメータ取得/設定（任意） ---
    # インベントリパラメータ取得コマンドを送信
    result = communicate(ser, COMMANDS['UHF_GET_INVENTORY_PARAM'])
    if re.match(STX + b'.' + ACK, result):
        print("UHF_GET_INVENTORY_PARAM が正常に実行されました")
    elif re.match(STX + b'.' + NACK, result):
        print(parse_nack_response(result))
    else:
        print("UHF_GET_INVENTORY_PARAM 実行エラー")
        print(result.hex())
        ser.close()
        sys.exit(1)

    # インベントリパラメータ設定コマンドを送信
    result = communicate(ser, COMMANDS['UHF_SET_INVENTORY_PARAM'])
    if re.match(STX + b'.' + ACK, result):
        print("UHF_SET_INVENTORY_PARAM が正常に実行されました")
    elif re.match(STX + b'.' + NACK, result):
        print(parse_nack_response(result))
    else:
        print("UHF_SET_INVENTORY_PARAM 実行エラー")
        print(result.hex())
        ser.close()
        sys.exit(1)

    # --- 読み取りループ ---
    total_read_time   = 0.0 # 総読み取り時間
    total_read_count  = 0   # 総読み取りタグ数
    total_iterations  = 0   # 総繰り返し回数
    pc_uii_count_dict = {}  # PC+UIIごとの読み取り回数を格納する辞書

    while True:
        try:
            repeat_count_str = input("繰り返す回数を入力してください（1〜100、終了は'q'）: ").strip()
            if repeat_count_str.lower() == 'q':
                break # 'q'が入力されたらループを終了
            repeat_count = int(repeat_count_str)
            if not (1 <= repeat_count <= 100):
                raise ValueError("1から100の範囲で入力してください")
        except ValueError as e:
            print(f"入力エラー: {e}。再度入力してください。")
            continue

        total_iterations += repeat_count

        start_time = time.time()
        for _ in range(repeat_count):
            # UHFインベントリコマンドを送信
            result = communicate(ser, COMMANDS['UHF_INVENTORY'])
            if result:
                # 受信データを解析し、PC+UIIリスト、RSSIリスト、期待される読み取り数を取得
                pc_uii_list, rssi_list, expected_count = received_data_parse(result)
                for pc_uii in pc_uii_list:
                    pc_uii_hex = pc_uii.hex().upper() # PC+UIIを16進数文字列に変換
                    print(f"PC+UII: {pc_uii_hex}")
                    pc_uii_count_dict[pc_uii_hex] = pc_uii_count_dict.get(pc_uii_hex, 0) + 1 # カウントを更新
                total_read_count += len(pc_uii_list)
            else:
                print("インベントリ応答がありませんでした。")

        end_time = time.time()
        total_read_time += (end_time - start_time)

        print(f"現在の合計読み取り時間: {total_read_time:.2f} 秒")
        print(f"現在の合計読み取り枚数: {total_read_count} 枚")

    # --- ブザー制御 ---
    # ブザーを鳴らす (ピッピッピ)
    print("ブザーを鳴らします (ピッピッピ)")
    buzzer_response = communicate(ser, COMMANDS['UHF_BUZZER_pipipi'])
    if re.match(STX + b'.' + ACK, buzzer_response):
        print("ブザー制御 ACK 受信")
    elif re.match(STX + b'.' + NACK, buzzer_response):
        print(parse_nack_response(buzzer_response))
    else:
        print("ブザー制御 ACK/NACK なし")

    time.sleep(1) # 1秒待機

    # ブザーを止める (ピー)
    print("ブザーを止めます (ピー)")
    buzzer_response = communicate(ser, COMMANDS['UHF_BUZZER_pi'])
    if re.match(STX + b'.' + ACK, buzzer_response):
        print("ブザー制御 ACK 受信")
    elif re.match(STX + b'.' + NACK, buzzer_response):
        print(parse_nack_response(buzzer_response))
    else:
        print("ブザー制御 ACK/NACK なし")

    # --- 集計結果の保存 ---
    save_results_to_file("inventory_results.txt", total_iterations, total_read_time, total_read_count, pc_uii_count_dict)
    print("集計結果を inventory_results.txt に保存しました。")

    # --- シリアルポートクローズ ---
    if ser and ser.is_open:
        ser.close()
    print("接続を閉じました。")

if __name__ == '__main__':
    main()

