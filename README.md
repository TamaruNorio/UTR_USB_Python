# Takaya UTR-S201/202 USB Sample (Python)

タカヤ製RFIDリーダライタ **UTR-S201/202 シリーズ** を  
USBシリアル接続で制御するための **Pythonサンプルプログラム** です。  
本プログラムは社内検証用に作成したものを公開しており、**無保証** での提供となります。

---

## 📦 動作環境
- OS: Windows 10 / 11  
- Python: 3.8 以上  
- 必要ライブラリ: [pyserial](https://pypi.org/project/pyserial/)  

インストール例:
```bash
pip install pyserial
```

---

## 🚀 使い方
1. UTR-S201/202 リーダライタをUSB接続  
2. ソースコードをダウンロードし、任意のフォルダへ展開  
3. コマンドプロンプト（またはPowerShell）から実行
   ```bash
   python utr_usb_sample.py
   ```
4. コンソールにタグ読取結果が表示されます  

---

## 📖 参考資料
本プログラムは **通信プロトコル仕様書** に基づいて作成されています。  
詳細仕様は以下を参照してください。  

- [UTRシリーズ 通信プロトコル仕様書](https://www.product.takaya.co.jp/rfid/download/uhf.html)  
- [制御用ソフト開発方法 (TDR-OTH-PROGRAMMING-103)](https://www.takaya.co.jp/product/rfid/)  

---

## ⚠️ 注意事項
- 本プログラムは **サンプル** です。商用利用を前提とした品質保証は行っていません。  
- ご利用にはRFID機器およびプロトコル仕様に関する基礎知識が必要です。  
- 実際の機器設定・運用には **UTR-RWManager** を併用してください。  

---

## 📄 ライセンス
このリポジトリのソースコードは **MIT License** の下で公開されています。  
詳細は [LICENSE](./LICENSE) ファイルをご確認ください。
