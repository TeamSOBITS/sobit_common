<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# SOBIT Common

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行・操作方法">実行・操作方法</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#変更履歴">変更履歴</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## 概要

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

SOBIT Commonのレポジトリでは、SOBITSがこれまで開発してきた実機ロボットを動かすため，共通のライブラリです．
ロボットに搭載されているアクチュエータやセンサーなど共通で使用するリソースを統一し，ライブラリ化となったものです．


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

### 環境条件

正常動作のため，以下の必要な環境を整えてください．


| System  | Version |
| ------------- | ------------- |
| Ubuntu  | 20.04 (Focal Fossa)  |
| ROS  | Noetic Ninjemys  |
| Python  | 3.0~  |


### インストール方法

1. ROSの`src`ファイルに移動します．
   ```sh
   $ roscd
   # roscdは設定によって`cd ~/catkin_ws/`のコマンドと同様
   $ cd src/
   ```

2. 本レポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/sobit_common
   ```
3. レポジトリの中へ移動します．
   ```sh
   $ cd sobit_common/
   ```
4. 依存パッケージをインストールします．ただし，移動機構によって`kobuki`か`custom`かが選択できます．
   ```sh
   $ bash install.sh --kobuki
   ```
5. パッケージをコンパイルします．
   ```sh
   $ roscd
   $ catkin_make
   ```

> [!IMPORTANT]  
> SOBITSでは`kobiki`ベースの移動機構があれば，自作の移動機構`custom`のロボットもあります．それに応じて，インストールするレポジトリが変わります．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- 実行・操作方法 -->
## 実行・操作方法
本レポジトリは共通ライブラリのため，実行すべきプログラムがありません．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- マイルストーン -->
## マイルストーン
- [ ] ドキュメンテーションの充実 
- [ ] OSS化
    - [ ] PCL関係のライブライをYOLOレポジトリへ移行
    - [ ] HSRの依存パッケージの差別化
    - [ ] 独自msgの更新

現時点のbugや新規機能の依頼を確認するために[Issueページ](https://github.com/github_username/repo_name/issues) をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- 変更履歴 -->
## 変更履歴

- 1.0: OSS (2023-11-06)
  - 詳細 1
  - 詳細 2
  - 詳細 3

<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- 参考文献 -->
## 参考文献

* [Dynamixel SDK e-Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Control](http://wiki.ros.org/ros_control)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_common.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_common/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_common.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_common/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_common.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_common/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_common.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_common/issues
<!-- [license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_common.svg?style=for-the-badge
[license-url]: https://github.com/TeamSOBITS/sobit_common/blob/master/LICENSE.txt -->