# akatsuki_au_bmfc21

[Design Plan](./Design-Plan.md)

## Setup

1. __Clone this repository__

1. __Install python and create virtual environment__

    - Follow this to install conda [link](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html)

    - Activate venv

    ```sh
    # create venv
    conda create -n brain
    conda activate brain

    # or using base
    conda activate
    ```

1. __Install required packages__

    ```sh
    cd Brain/
    pip3 install -r requirements_rpi.txt
    ```

    if installing on computer and not pi
    comment `picamera` and `RTIMULib` in [requirements_rpi.txt](./Brain/requirements_rpi.txt)

    ```
    # RTIMULib
    # picamera
    ```

1. __Setup Config__

    Update [config.json](./Brain/config.json) as required.

1. __Start main process__

    ```sh
    python3 go.py
    ```
