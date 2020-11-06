
# define the name of the virtual environment directory
VENV := venv

# default target, when make executed without arguments
all: venv

$(VENV)/bin/activate: requirements.txt
	make -C ./libcluon
	python3 -m venv $(VENV)
	./$(VENV)/bin/pip install --upgrade pip
	./$(VENV)/bin/pip install --editable .
	./$(VENV)/bin/pip install -r requirements.txt

# venv is a shortcut target
venv: $(VENV)/bin/activate

run_evaluate:
	./$(VENV)/bin/python3 ./main/computer_vision.py
	./$(VENV)/bin/python3 ./main/execution.py

run_hardware:
	./$(VENV)/bin/python3 ./main/computer_vision.py
	./$(VENV)/bin/python3 ./main/training.py -l True

run_software:
	./$(VENV)/bin/python3 ./main/training.py -l False

clean:
	rm -rf ./$(VENV)
	rm -rf ./hubert.egg-info
	find -name "*.pyc" -delete
	make clean -C ./libcluon