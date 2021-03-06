# Python オブジェクト指向（再）(2)

[python/oop/Home](Home.md)

---

## 実習(1)

- 社員を表す`Employee`クラスを実装します。
- ホームディレクトリ直下に演習用ディレクトリを作成する。

```shell
$ cd
$ mkdir -p python/oop
$ cd python/oop
```

- 作成したディレクトリに次のベースファイルをダウンロードする。
  - [app.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/python/oop/python_oop_02/app.py)
  - [employee.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/python/oop/python_oop_02/employee.py)

## 問題(1)

- `Employee`クラスに月給を表す`salary`フィールドを新たに作成しなさい。
- 初期値は`230000`となるようにコンストラクタに引数を加えなさい。

## 問題(2)

- `Employee`クラスに月給に基づいてボーナスを計算する`get_bonus`メソッドを作成しなさい。引数は実数の`month`。返却値は実数で`salary`×`month`で計算される賞与額を返す。
- `Employee`クラスに入社何年目かを計算する`get_experience`メソッドを作成しなさい。引数は整数の`year`。返却値は整数で`year`から`join.year`を引いた年数を返す。
- `Employee`クラスに以下の形式で社員情報を表示する`show`メソッドを作成しなさい。

```text
社員名：工大　太郎, 社員番号：x18-000, 部署：カスタマーサポート, 月給：230000, 賞与（6ヶ月）：1380000, 経験年数（2018年時点）：**年, 年齢：（2018年時点）：**歳
```

## 問題(3)

- `app.py`において、辞書を使い社員番号を使って社員を検索するプログラムを作成しなさい。社員数は 3 名でよい。
  - `app.py`で 3 名分の社員が入った辞書を作成し、任意の社員番号を持つ社員が辞書中にあればその社員のデータを表示し、なければエラーメッセージ（何でも良い）を表示する。

---

[python/oop/Home](Home.md)
