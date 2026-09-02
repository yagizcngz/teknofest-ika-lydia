# LYDIA IKA -- ROS 2 Build & Test Automation
# Kullanim: make <hedef>

.PHONY: build test harita arac clean lint format install-deps unit-test

# --- Degiskenler ---
PACKAGE       := teknofest_ika
WORKSPACE     ?= $(shell pwd)/../..
PYTHON        := python3
PIP           := pip3

# --- Temel Komutlar ---

build:
	@echo "==> colcon build --packages-select $(PACKAGE)"
	# --symlink-install YOK: aractaki install/ kopya tabanli, bayrakla derlemek
	# maps/teknofest_harita.pgm uzerinde [Errno 2] verip install alanini bozar.
	cd $(WORKSPACE) && colcon build --packages-select $(PACKAGE)

test:
	@echo "==> colcon test --packages-select $(PACKAGE)"
	cd $(WORKSPACE) && colcon test --packages-select $(PACKAGE)
	@echo "==> Test sonuclari:"
	cd $(WORKSPACE) && colcon test-result --verbose

harita:
	@echo "==> Haritalama modu baslatiliyor..."
	ros2 launch $(PACKAGE) gercek_harita.launch.py

arac:
	@echo "==> Gercek arac modu baslatiliyor..."
	ros2 launch $(PACKAGE) gercek_arac.launch.py

clean:
	@echo "==> build/ install/ log/ temizleniyor..."
	cd $(WORKSPACE) && rm -rf build install log

# --- Kod Kalitesi ---

lint:
	@echo "==> flake8 calistiriliyor..."
	flake8 $(PACKAGE) --count --select=E9,F63,F7,F82 --show-source --statistics
	flake8 $(PACKAGE) --count --exit-zero --max-complexity=15 --max-line-length=120 --statistics

format:
	@echo "==> black + isort calistiriliyor..."
	black $(PACKAGE) scripts test_birim.py || echo "black kurulu degil: pip install black"
	isort $(PACKAGE) scripts test_birim.py || echo "isort kurulu degil: pip install isort"

# --- Ortam Kurulumu ---

install-deps:
	@echo "==> Python bağımlılıkları kuruluyor..."
	$(PIP) install -r requirements.txt
	@echo "==> ROS 2 bağımlılıkları rosdep ile kuruluyor..."
	cd $(WORKSPACE) && rosdep install --from-paths src --ignore-src -y

# --- Hizli Test (donanim gerektirmez) ---

unit-test:
	@echo "==> Birim testleri calistiriliyor..."
	cd $(WORKSPACE)/src/$(PACKAGE) && $(PYTHON) test_birim.py

# --- Yardim ---

help:
	@echo "LYDIA IKA Makefile Hedefleri"
	@echo "  build        - colcon build (--symlink-install YOK, arac icin)"
	@echo "  test         - colcon test + sonuc raporu"
	@echo "  unit-test    - Donanim gerektirmeyen birim testleri"
	@echo "  harita       - Haritalama modu baslat"
	@echo "  arac         - Gercek arac modu baslat"
	@echo "  clean        - build/install/log temizle"
	@echo "  lint         - flake8 ile kod kalite kontrolu"
	@echo "  format       - black/isort ile kod formatla"
	@echo "  install-deps - requirements.txt + rosdep bağımlılıklarını kur"
