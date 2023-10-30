# starlite-raspi

Code that runs on the Raspberry Pi for the Starlite project.


## Before running any scripts & installing dependencies

Before running any scripts or installing dependencies on the repository, you need to activate the virtual environment:

- On Windows: `.\venv\Scripts\activate`
- On Linux: `venv/bin/activate`

To exit the virtual environment:
`deactivate`


## Installing dependencies

To install all dependencies on the virtual environment:

`python -m pip install -r requirements.txt`


## Updating dependencies

To update the dependencies of this repository:

`python -m pip freeze > requirements.txt`


## Running the scripts

When the virtual environment is active, and the dependencies already installed, the scripts are ready to run.


## More information

For more information on Python's virtual environment:

[https://realpython.com/python-virtual-environments-a-primer/](https://realpython.com/python-virtual-environments-a-primer/)