PYTHON ?= python

.PHONY: install lint test build audit firmware check

install:
	$(PYTHON) -m pip install -r requirements.lock
	$(PYTHON) -m pip install --no-deps -e .

lint:
	$(PYTHON) -m ruff format --check tests/test_report_provenance.py rxconfig.py run_app.py
	$(PYTHON) -m ruff check .

test:
	$(PYTHON) -m pytest -q

build:
	$(PYTHON) -m build --no-isolation

audit:
	$(PYTHON) -m pip_audit -r requirements.lock

firmware:
	$(PYTHON) -m platformio run

check: lint test build
