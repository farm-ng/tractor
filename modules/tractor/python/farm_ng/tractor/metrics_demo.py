import logging
import random
import sys
import time

from prometheus_client import Counter
from prometheus_client import Gauge
from prometheus_client import Summary
from prometheus_client import start_http_server

logging.basicConfig(stream=sys.stdout)
logger = logging.getLogger('metrics_demo')
logger.setLevel(logging.INFO)

cycle_time = Summary('cycle_time', 'Cycle time')
lifetime_odometer = Counter('lifetime_odometer', 'Meters driven since epoch')
voltage = Gauge('battery_voltage', 'Battery voltage')
current = Gauge('battery_current', 'Battery current')
energy_usage = Counter('battery_energy_usage', 'Energy usage, in Ws')


@ cycle_time.time()
def process_request():
    t = random.random()
    time.sleep(t)
    lifetime_odometer.inc(random.random())
    v = random.random() + 27
    a = random.random()
    voltage.set(v)
    current.set(a)
    energy_usage.inc(v * a * t)
    logger.info('Cycle')


if __name__ == '__main__':
    start_http_server(7000)
    while True:
        process_request()
