<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS Common

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#launch-and-usage">Launch and Usage</a></li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- INTRODUCTION -->
## Introduction

The SOBITS Common repository is a common library for operating robots developed by SOBITS. In this library you will be able to collect information about the actuators and sensors mounted on these robots.

The robots that require SOBITS Common are listed here.

| SOBIT PRO | SOBIT EDU | SOBIT MINI |
| :---: | :---: | :---: |
| ![SOBIT PRO](docs/img/sobit_pro.png) | ![SOBIT EDU](docs/img/sobit_edu.png) | ![SOBIT MINI](docs/img/sobit_mini.png) | 
| [Go to Git](https://github.com/TeamSOBITS/sobit_pro) | [Go to Git](https://github.com/TeamSOBITS/sobit_edu) | [Go to Git](https://github.com/TeamSOBITS/sobit_mini) |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

Here you will find out instructions on setting up this project locally.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBIT Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of ROS.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ cd src/
   ```

2. Clone this repository.
   ```sh
   $ git clone https://github.com/TeamSOBITS/sobits_common
   ```
3. Navigate into the repository.
   ```sh
   $ cd sobits_common/
   ```
4. Install the dependent packages.
   ```sh
   $ bash install.sh
   ```
5. Compile the package.
   ```sh
   $ roscd
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LAUNCH AND USAGE EXAMPLES -->
## Launch and Usage

This repository is a common library, so there are no programs to run.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MILESTONE -->
## Milestone

- [x] OSS
    - [x] Migrate PCL-related libraries to YOLO repository
    - [x] Isolation of HSR dependency packages
    - [x] Improve documentation 
    - [ ] Update custom messages

See the [open issues][issues-url] for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


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

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [Dynamixel SDK e-Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Control](http://wiki.ros.org/ros_control)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



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
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_common.svg?style=for-the-badge
[license-url]: LICENSE
