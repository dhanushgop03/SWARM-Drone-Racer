# SWARM-Drone-Racer


# AirSimNeurIPS API Setup Guide

This guide will help you set up your Python environment and install the necessary dependencies to run the AirSimNeurIPS API and simulations.

## 1. Check Python Version

Before proceeding, ensure that you have Python version > 3.6 installed on your system (python 3.10 is supported). You can check your Python version by running the following command in your terminal or command prompt:

```bash
python --version
```

Make sure the output shows Python 3.10. If you do not have Python 3.10 installed, you will need to install it before proceeding.

## 2. *Optional: Create a Virtual Environment

It's recommended to create a virtual environment to keep your dependencies isolated. To create and activate a virtual environment, use the following commands:

```bash
python -m venv airsimvenv
```

Activate the virtual environment:

- On **Windows**:
  ```bash
  .\airsimvenv\Scripts\activate
  ```

- On **Linux/MacOS**:
  ```bash
  source airsimvenv/bin/activate
  ```

## 3. Install Python Packages from `requirements.txt`

With the virtual environment activated (if you created one), install the required Python packages by running:

```bash
pip install -r requirements.txt
```

This will install all the dependencies listed in the `requirements.txt` file.

## 4. Prepare to Run the Simulation

Now that your environment is set up, you are ready to run the AirSimNeurIPS API. Execute the simulation by running the following command:

```bash
./AirsimExe.exe -windowed
```

This will start the simulation in a windowed mode.

## 5. Run Your Script

Finally, you can run your Python script to interact with the simulation. Replace `flighttest.py` with the name of your script:

```bash
python flighttest.py
```

Your setup is complete, and you are now ready to run your simulations with the AirSimNeurIPS API.
