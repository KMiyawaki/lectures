# Three.js(準備 2)

[three_js/Home](./Home.md)

- - -

## JavaScript の基本

Visual Studio Code で`ThreeJS-master/lec01/base10.html`を開き、HTML ファイルおよび JavaScript プログラムの流れを理解する。  
ここでは HTML の詳細な説明は省略するが、最低限以下の点を確認すること。

- HTML の要素は次の例のように開始タグから始まり、終了タグで終わる。開始タグにはタグの挙動を制御する様々な属性が記述される。

```html
<!-- ある div 要素の開始タグ -->
<div id="screen" class="screen">
  <!-- 属性は 属性名="値" の形で列挙される -->
  子要素
</div>
<!-- スラッシュ付きが終了タグ -->
```

- `style=`で始まる属性は主に要素の見栄えをコントロールしている。位置や大きさ、背景色、背景画像などである。

```html
<div id="glView" style="position: relative;height: 75%;"></div>
<!-- relative は直前に表示した要素の位置を基準として順番に表示する場合に指定する。height は親要素の高さに対して自分自身の大きさをどの程度の割合にするかを示す -->
```

- `style`属性の定義が長くなったり、同じ`style`を使いまわしたりするとき、`style`定義を外部ファイルに記述できる。
- 外部ファイルの拡張子`.css`はカスケーディングスタイルシートの略である。

`base10.html`の冒頭、以下の部分で CSS ファイルを読込んでいる。

```html
<link rel="stylesheet" type="text/css" href="../css/common.css" />
```

- `ThreeJS-master/css/common.css`には様々なスタイルを名前を付けて定義しており、HTML からは次の例のように要素に`class`属性をつけてスタイル名を指定し呼び出すことができる。

```html
<textarea id="debugText" rows="12" cols="30" class="debugText">
<!-- この textarea には debugText というスタイルを適用する -->
Debug output
</textarea>
```

- `<!--`～`-->`で囲まれた部分はコメントである。

次に JavaScript 部分について説明する。まず別ファイルに記述された JavaScript を以下の HTML タグにより読み込んでいる。

```html
<!-- Three.js の本体 -->
<script type="text/javascript" src="https://cdn.jsdelivr.net/gh/mrdoob/three.js@r114/build/three.min.js"></script>
<!-- マウスでカメラを操作するライブラリ。Three.js に依存している -->
<script type="text/javascript"
    src="https://cdn.jsdelivr.net/gh/mrdoob/three.js@r114/examples/js/controls/OrbitControls.js"></script>
<!-- 演習用に用意した関数群 -->
<script type="text/javascript" src="../js/myThree2020.js"></script>
<script type="text/javascript" src="../js/myGameCharacter.js"></script>
```

以上で取り込んだ JavaScript のクラス、関数群を利用し、自分のプログラムを次のように記述している。

```html
<script type="text/javascript">
  function init() {
    ・・・
  }
  window.addEventListener('load', init);
</script>
```

演習用のプログラムの主要な部分はこの`init`関数内に記述する。C 言語でいうところの`main`関数と似ている。ゲーム制作にあたり、キャラクターを表現するクラスなどは外部ファイルに記述し、`<script type="text/javascript" src=`によって取り込む。

### 変数

#### 基本型

- 文字列（string）
- 数値（number）
- 真偽（boolean）

#### それ以外の型

- 連想配列、配列、クラス（厳密には連想配列の一種）

JavaScript の変数は宣言時に型を書く必要はない。代入された値によって型が決まる。ただし、特定のキーワードをつける必要がある。

- `let`：値の再代入が可能な変数。スコープはブロック内限定。ループカウンタや計算結果の一時保存などに使える。
- `const`：値の再代入が**不可能**な定数。スコープはブロック内限定。C 言語の`#define`と似たような使い方や、無意味なインスタンス生成を防止できる。
- `var`：は WEB 上の JavaScript 関係情報で頻出しているが、スコープがブロックに限定されないので、本演習ではなるべく使用しない。

変数定義の例を下記に示す。仮に以下のプログラムを実行した場合、「エラー」と書かれた部分のコメントアウトを解除すると JavaScript はそこで動作を停止し、ブラウザの表示が意図した通りにならなくなる。PC の Chrome であれば F12 キーを押すことでエラー内容を表示できる。[前項](./three_js_prep_01.md)を参照。

```javascript
const NUM = 4; // 数値型の定数
const C_STRING = "This is const string."; // 文字列型の定数
let x = 50; // 数値型の変数
const array = [1, 2, 3, 4, 5]; // 再代入不可能な配列型
x = 26;
// let x = 30; // エラー。同名の変数の宣言を防げる。
// NUM = 8; // エラー。定数に値は代入できない。
// C_STRING = "change"; // エラー。定数に値は代入できない。
array[1] = 99; // OK。配列の中身は変更可能。
array.push(20); // OK。値の追加も可能。 push は末尾に追加する。
array.splice(1, 1); // OK。値の削除も可能。 splice は第一引数で指定した添え字から、第二引数で指定した個数の要素を削除する。
array.splice(2, 0, 10); // OK。値の挿入も可能。 splice は第一引数で指定した添え字の箇所に第三引数以降の値を挿入できる。第二引数にゼロを指定すれば挿入だけができる。
// array = [9, 8, 7]; // エラー。新しい配列を作って代入することはできない。
let count = array.length; // 配列の長さを得ることもできる。
const array2 = [3, 2, 1, 2, 3]; // 再代入不可能な配列型
let index_2f = array2.indexOf(2); // 要素を先頭から検索し、その添え字を返す。
let index_2l = array2.lastIndexOf(2); // 要素を末尾から検索し、その添え字を返す。
let index_na = array2.indexOf(99); // 要素がなければ -1 を返す。
let pos = C_STRING.indexOf("is"); // pos には 2 が入る。文字列でもindexOfは使える。部分一致のチェックができる。
```

### 連想配列

検索が可能なデータ構造。検索用の「キー」と特定の「キー」に対応付いた「値」のペア（キー・バリューペアともいう）が複数格納されている。他言語では「辞書」呼ばれていることもある。  
キーと値は「:（コロン）」で区切る。キーと値には様々な型を代入可能。一つの連想配列で異なる型の値を利用することもできる。

```javascript
const dict = { x: 85, y: 12, z: 33 }; // 連想配列の宣言。
let valueX = dict["x"]; // valueX には 85 が入る。
dict["z"] = 120; // dict の中身は { x: 85, y: 12, z: 120 }; となる。
let valueW = dict["w"]; // valueW には undefined（後述）が入る。 w というキーは無い。
if(dict["y"]){ // キーがあるかどうかは簡単な if 文でチェックできる。
  // キー y が存在する場合の処理
}
```

連想配列の特定のキーを持つ値へのアクセスは次のように「.（ドット演算子）」で書くこともできる。
ただし、数字が先頭に来ていたり、途中にスペースがあったりするなど、変数名として適切でないキーの場合はこの「.（ドット演算子）」による値へのアクセスはできないので、`[]`の演算子を使う。  
下記の記述方法は他言語でのクラスや構造体を JavaScript で実現するために重要な記述方法である。詳細は後述する。

```javascript
let valueY = dict.y; // dict のキー y に対応する値を valueY に代入する。 12 が入る。
dict.y = 30; // dict の中身は { x: 85, y: 30, z: 120 }; となる。
```

連想配列への値の追加や削除は次のようにする。

```javascript
dict["p"] = 128; // dict の中身は { x: 85, y: 12, z: 120, p:128 }; となる。
dict.q = 255; // dict の中身は { x: 85, y: 12, z: 120, p:128, q:255 }; となる。
delete dict.z; // dict の中身は { x: 85, y: 12, p:128, q:255 }; となる。
```

### if 文

他言語とほぼ同じ。等価、比較、論理の演算子も同様に使える。

```javascript
let x = 8;
if (x > 5 && x < 10) {
  ...
} else {
  ...
}
if (x % 2 == 0) {
  ...
} else {
  ...
}
let str = "Hello";
if(str == "Hello"){ // 文字の比較も可能。部分一致をチェックしたいなら、indexOfを使う。
  ...
}else{
  ...
}
```

#### 厳密等価/不等価

JavaScript の比較には型が一致するかどうかまでチェックする演算子がある。厳密等価（`===`）と厳密不等価（`!==`）である。

```javascript
let result;
result = ("1" == 1); // result は true。同じ型になるようにキャストされてから比較される。
result = ("1" === 1); // result は false。型が違う。
```

本演習では、 C 言語など基礎的な言語で慣れている通常の等価（`==`）/不等価（`!=`）演算子で進める。

### for 文

ループカウンタを使う場合は他言語とほぼ同じ。

```javascript
const array = [1, 2, 3, 4, 5]; // 再代入不可能な配列型
for (let i = 0; i < array.length; i++) {
  let val = array[i];
}
```

配列の場合は`of`キーワードを使って要素を順番に参照できる。

```javascript
const array = [1, 2, 3, 4, 5]; // 再代入不可能な配列型
for (let val of array) {
  /* val には array の要素が順に入る。 */
}
```

連想配列の場合は`in`キーワードを使ってキーを順番に参照できる。

```javascript
const dict = { x: 85, y: 12, z: 33 }; // 連想配列の宣言。
for (let k in dict) {
  /* k には dict のキー（x, y, z）が順に入る。 */
  let val = dict[k]; /* val には k に対応する値が入る。 */
}
```

### 関数定義

下記の`myAdd`、`mySub`どちらの方法でも関数を定義することができる。

```Javascript
function init(){
  /* 省略 */
    // 関数内に関数を定義することもできる。
    // 関数名 myFunc、引数は arg1, arg2 の 2 つ。
  function myAdd(arg1, arg2){
    let ans = arg1  + arg2;
    return ans;
  }
  const mySub = function (arg1, arg2) {
    return arg1 - arg2;
  }

  let result = myAdd(5, 2); //
  result = mySub(6, 10); // 関数の呼び出し。

  function makeArray(arg1, arg2, arg3){
    return [arg1, arg2, arg3]; // 配列も返却できる。
  }
  function makeDict(key1, key2, value1, value2){
    const result = {};
    result[key1] = value1;
    result[key2] = value2;
    return result;// 連想配列も返却できる。
  }
 /* 省略 */
}
```

`mySub`の記述方法は関数式と呼ばれる。この書き方は、ある関数全体が定数`mySub`に格納されているという様子がイメージできる。定数に格納されているから`mySub`という名前で呼び出せるのであって、名前をつけずに関数を定義することもできる。これを無名関数といい、GUI のイベント処理などを記述する際によく利用する。  
`base10.html`の下の方にある、画面サイズ更新時の処理が無名関数を利用している。

```javascript
/* 画面サイズ変更時の処理 */
window.addEventListener("resize", function() {
  taDebugText.value += "ViewPort: " + window.innerWidth + "," + window.innerHeight + "\n";
・・・
});
```

### クラス

RPG のキャラクターを表現するクラス`MyGameCharacter`の一部を JavaScript と Java でほぼ同じになるように記述してみる。  
[完全版](https://github.com/KMiyawaki/ThreeJS/blob/master/js/myGameCharacter.js)も確認しておくこと。

- JavaScript

```javascript
class MyGameCharacter {
  constructor(name, imageURL, hp, power, defense, speed) {
    this.name = name; // 名前
    this.hp = hp; // 体力
    this.power = power; // 腕力
    this.defense = defense; // 守備力
    this.speed = speed; // 素早さ
    this.imageURL = imageURL; // 画像の URL
  }

  /* このキャラクタが生存しているかどうかの判定。 */
  isAlive() {
    return this.hp > 0;
  }

  /* 他のキャラクタに攻撃が命中するかどうかを判定する。 */
  checkHit(targetCharacter) {
    return Math.random() * this.speed > Math.random() * targetCharacter.speed;
  }
}
```

- Java

```java
public class MyGameCharacter {
  public String name; // 名前
  public int hp; // 体力
  public int power; // 腕力
  public int defense; // 守備力
  public int speed; // 素早さ
  public String imageURL; // 画像の URL

  MyGameCharacter(String name, String imageURL, int hp, int power, int defense, int speed) {
    this.name = name;
    this.hp = hp;
    this.power = power;
    this.defense = defense;
    this.speed = speed;
    this.imageURL = imageURL;
  }

  /* このキャラクタが生存しているかどうかの判定。 */
  boolean isAlive() {
    return this.hp > 0;
  }

  /* 他のキャラクタに攻撃が命中するかどうかを判定する。 */
  boolean checkHit(MyGameCharacter targetCharacter) {
    return Math.random() * this.speed > Math.random() * targetCharacter.speed;
  }
}
```

コンストラクタやフィールドの定義場所に若干の違いがあるが、ほぼ同じような記述が可能である。上記のように定義したクラスからは次のように`new`キーワードによってインスタンス生成できる。

```javascript
const hero = new MyGameCharacter("主人公", null, 15, 5, 7, 8);
let monster = new MyGameCharacter("ドラゴン", "../assets/downloads/red-dragon-1549047184nu3.png", 30, 20, 4, 3);
...
```

なお、JavaScript においてはクラスのインスタンスは連想配列である。上記の`hero`は次のような連想配列にクラスのメソッド群が組み合わさっているだけである（実際はメソッド群も連想配列のキーバリューペアとして組み込まれている）。

```javascript
{ name: "主人公", imageURL: null, hp: 15, power: 5, defense: 7, speed: 8 }
```

したがって、クラスのインスタンスに格納されている値やメソッドは連想配列と同じく次のようにして参照できる。

```javascript
const herosName = hero.name; // herosName には「主人公」が入る。
hero.power += 20; //　腕力アップ。
if (hero.isAlive()) {
  /* 生きてる！ */
}
if (hero.checkHit(monster) == false) {
  /* かわされた！ */
}
```

（補足）C 言語における構造体は連想配列の機能を使えば似たような機能が簡単に実現できる。連想配列のキーが構造体のメンバ名に相当している。

### null

`null`は変数や定数が何も参照していないことを示す。クラスや配列、連想配列を格納する変数の初期値としてよく使う。Java の`null`と似ている。また、関数やクラスのメソッドの引数に与え、「無効な値」を表現する際にも使える。  
次の例でコンストラクタの第二引数に`null`が指定されている。

```javascript
const hero = new MyGameCharacter("主人公", null, 15, 5, 7, 8);
...
```

ここで、インスタンス生成後`hero.imageURL`の中身は`null`となっている。理由はコンストラクタの定義を見れば分かる。仮引数`imageURL`に`null`が与えられ、そのまま`this.imageURL`に代入されているからである。

```javascript
constructor(name, imageURL, hp, power, defense, speed) {
  this.name = name; // 名前
  this.hp = hp; // 体力
  this.power = power; // 腕力
  this.defense = defense; // 守備力
  this.speed = speed; // 素早さ
  this.imageURL = imageURL; // 画像の URL
}
```

`MyGameCharacter`の[完全版](https://github.com/KMiyawaki/ThreeJS/blob/master/js/myGameCharacter.js)では`this.imageURL`が`null`でない場合だけこのキャラクタの[画像をロードして表示する](https://github.com/KMiyawaki/ThreeJS/blob/master/js/myGameCharacter.js#L72)というような使い方をしている。

### undefined

`undefined`は変数や定数が定義されていないことを示す。例えば定義されていない変数を関数の引数に与えたとき、関数内部では`undefined`となっており、大抵の場合エラーの原因となる。連想配列（やクラス）で存在しないキーを検索しようとした際も同様の値が返却される。

```javascript
const hero = new MyGameCharacter("主人公", null, 15, 5, 7, 8);
let monster = new MyGameCharacter("ドラゴン", "../assets/downloads/red-dragon-1549047184nu3.png", 30, 20, 4, 3);

if (hero.checkHit(Monster) == false) { // Uncaught ReferenceError: Monster is not defined というエラーが発生する。
}
...
```

### HTML 要素の取得

`document.getElementById("HTML要素のid")`で HTML の要素を取得することができる。以下のような HTML の記述を想定する。`id`属性が指定してあることが条件である。

```html
<textarea id="debugText" rows="8" cols="20" class="debugText">
Debug output
</textarea>
```

これに対し、次のようなスクリプトを書く。

```javascript
const taDebugText = document.getElementById("debugText");
```

以降 taDebugText は HTML の textarea 要素を保持し、その内容を動的に変更できるようになる。つまり、ブラウザの表示内容をプログラムからコントロールできる。`base10.html`では、`textarea`の`value`属性（string）に代入したり、追加したりすることでその表示内容を制御している。

```java
/* デバッグ用の出力 */
taDebugText.value = "ViewPort: " + window.innerWidth + "," + window.innerHeight + "\n";
```

他の例としては、`div`要素を変数に保持しておいて`innerHTML`を使い`div`内に文字を出現させることもできる。

```html
<div id="test"></div>
<script type="text/javascript">
  function init() {
    const divTest = document.getElementById("test");
    divTest.innerHTML = "<b>Hello Javascript</b>"; // 太字の文字がブラウザに表示される。
  }
</script>
```

## 参照

参照の概念は Java と似ている。JavaScript の連想配列、配列、クラスといった基本型**以外**の型の変数は値への参照を保持している。

```javascript
class Person{
  constructor(name, age){
    this.name = name;
    this.age = age;
  }
}
```

というクラスがあったとして、下記のように利用することを考える。

```javascript
const p1 = new Person("Taro", 23);
const pRef = p1;
pRef.name = "Hanako";
/* p1.name を出力すると Hanako となっている */
```

ここで、変数`pRef`と`p1`はメモリ上の同じ個所を指している。これを「参照している」と言う。
`pRef.name = "Hanako";`とすると、メモリ上にある`Person`のデータ`name`が変更される。  
`pRef`と`p1`は同じものを参照しているのだから、`p1.name`の出力結果も`Hanako`となる。  

関数やメソッドの仮引数として連想配列、配列、クラスを利用する場合、上記のようにメモリ上の実体への参照が渡される。
よって、関数やメソッド内部で仮引数の値を書き換えた場合、関数呼び出し側の実引数が参照する値も変更されている。

一方、基本型の場合は変数は値そのものを保持している。そのため、関数内部で仮引数を書き換えても呼び出し側の実引数には影響はない。

- 検証用コード

```html
<!DOCTYPE html>
<html>
<title>Web Page Design</title>
<head>
<script>
class Person{
    constructor(name, age){
        this.name = name;
        this.age = age;
    }
}
function changeName(p, name){
    p.name = name;
}
function changeArray(a){
    a[0] = 99;
}
function changeInt(i){
    i = 200;
}

function init() {
    const p1 = new Person("Taro", 23);
    changeName(p1, "Hanako");
    const array = [1, 2, 3];
    changeArray(array);
    let x = 15;
    changeInt(x);
    document.write(p1.name); // document.write でWEBページに直接HTMLを出力できる。
    document.write("<br/>");
    document.write(array);
    document.write("<br/>");
    document.write(x);
}
init();
</script>
</head>
<body>
</body>
</html>
```

- 出力

```text
Hanako
99,2,3
15
```

## 参考

- [codingground](https://www.tutorialspoint.com/online_javascript_editor.php)
  - ブラウザで手軽に JavaScript のコードが試せる。

- - -

[three_js/Home](./Home.md)
