import asyncio
import logging
import re
import sys

logger = logging.getLogger('rtkcli')
logger.setLevel(logging.INFO)

_status_map = {
    '1': 'fix', '2': 'float', '3': 'sbas',
    '4': 'dgps', '5': 'single', '6': 'ppp',
}


def _status_mapper(value):
    return _status_map[value]


class RtkClient:
    # % (e/n/u-baseline=WGS84,Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,ns=#
    # %  of satellites) UTC e-baseline(m) n-baseline(m) u-baseline(m)
    # %  Q ns sde(m) sdn(m) sdu(m) sden(m) sdnu(m) sdue(m) age(s)
    # %  ratio ve(m/s) vn(m/s) vu(m/s) sdve sdvn sdvu sdven sdvnu
    # %  sdvue

    field_names = [
        ('date', str),
        ('time_gps_utc', str),
        ('e_baseline_m', float),
        ('n_baseline_m', float),
        ('u_baseline_m', float),
        ('status',  _status_mapper),
        ('n_satelites', int),
        ('std_e_m', float),
        ('std_n_m', float),
        ('std_u_m', float),
        ('sde_en_m', float),
        ('sde_nu_m', float),
        ('sde_ue_m', float),
        ('age', float),
        ('ar_ratio', float),
        ('velocity_e_ms', float),
        ('velocity_n_ms', float),
        ('velocity_u_ms', float),
        ('std_ve', float),
        ('std_vn', float),
        ('std_vu', float),
        ('std_ven', float),
        ('std_vnu', float),
        ('std_vue', float),
    ]

    def __init__(self, rtkhost, rtkport, rtktelnetport, event_loop=None):
        self.rtkhost = rtkhost
        self.rtkport = rtkport
        self.rtktelnetport = rtktelnetport

        self.n_states = 10000
        self.n_status_messages = 100
        self.status_messages = []
        self.gps_states = []
        self.event_loop = event_loop
        if self.event_loop is not None:
            self.event_loop.create_task(self.run())
            self.event_loop.create_task(self.run_telnet())

    async def connect(self):
        self.reader, self.writer = await asyncio.open_connection(
            self.rtkhost, self.rtkport,
        )

    async def read_one(self):
        message = await self.reader.readline()
        message = message.rstrip(b'\n').decode('ascii')
        fields = re.split(r'\s+', message)
        # expecting fields to be the correct number,
        # TODO(ethanrublee) better input checking...
        assert len(fields) == len(RtkClient.field_names), '%d != %d' % (
            len(fields), len(RtkClient.field_names),
        )

        # create a dict of field names to field values
        gps_state = {
            key: mapf(value) for (key, mapf),
            value in zip(RtkClient.field_names, fields)
        }
        self.gps_states.append(gps_state)
        logger.debug(gps_state)
        if len(self.gps_states) > self.n_states:
            self.gps_states.pop(0)

    async def connect_and_read_loop(self):
        await self.connect()
        while True:
            await self.read_one()

    async def run(self):
        while True:
            try:
                await self.connect_and_read_loop()
            except OSError as e:
                logger.error(e)
                await asyncio.sleep(1)

    async def connect_and_read_telnet_loop(self):
        reader, writer = await asyncio.open_connection(
            self.rtkhost, self.rtktelnetport,
        )
        while True:
            logger.info(
                'connected to rtkrover telnet host: %s',
                self.rtkhost, self.rtktelnetport,
            )
            message = await reader.readuntil(b'password: ')
            message = message.strip(b'\xff\xfb\x03\xff\xfb\x01\r\n\x1b[1m')
            logger.info(message.decode('ascii'))
            await writer.write(b'admin\r\n')
            message = await reader.readuntil(b'\r\n')
            logging.info(message.decode('ascii'))
            yield writer.write(b'status 1\r\n')
            while True:
                message = yield reader.readuntil(b'\x1b[2J')
                status_msg_ascii = escape_ansi(
                    message.rstrip(b'\x1b[2J').decode('ascii'),
                )
                self.status_messages.append(status_msg_ascii)
                logger.debug(status_msg_ascii)
                if len(self.status_messages) > self.n_status_messages:
                    self.status_messages.pop(0)

    async def run_telnet(self):
        while True:
            try:
                await self.connect_and_read_telnet_loop()
            except OSError as e:
                logger.error(e)
                await asyncio.sleep(1)


def main():
    rtk_client = RtkClient('localhost', 9797, 2023)
    loop = asyncio.get_event_loop()
    # Blocking call which returns when the hello_world() coroutine is done
    loop.run_until_complete(rtk_client.run())
    loop.close()


if __name__ == '__main__':
    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
    main()
