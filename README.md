# sheep-2021-emulator
git clone https://github.com/trygve55/sheep-2021-emulator

git submodule update --init --recursive

python3 -m venv .

source bin/activate

pip install lxml future pyserial

cd mavlink

python -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=pymavlink/dialects/v20/sheeprtt.py message_definitions/v1.0/sheeprtt.xml

python -m pymavlink.tools.mavgen --lang=Python --wire-protocol=1.0 --output=pymavlink/dialects/v10/sheeprtt.py message_definitions/v1.0/sheeprtt.xml

cd pymavlink

python setup.py install
