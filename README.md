# starlite-raspi

Code that runs on the Raspberry Pi for the Starlite project.


## Initial Setup

1. Create the virtual environment: `python -m venv venv`

2. Activate the virtual environment:
  
    - On Windows: `.\venv\Scripts\activate`
    - On Linux: `venv/bin/activate`

3. Install all dependencies on the virtual environment: `python -m pip install -r requirements.txt`


## Running the scripts

Make sure that the virtual environment is active and the dependencies already installed before running any scripts.


## Deactivating the virtual environment

To exit the virtual environment when the virtual environment is active: `deactivate`


## Updating dependencies

Make sure the virtual environment is active. To update the dependencies of this repository:

`python -m pip freeze > requirements.txt`


## More information

For more information on Python's virtual environment:

[https://realpython.com/python-virtual-environments-a-primer/](https://realpython.com/python-virtual-environments-a-primer/)