"""Left relay control via verified raw Modbus RTU frames, preserving end power."""
from .control_trace import invoke


class SuctionError(RuntimeError):
    pass


def validate_config(config):
    if not isinstance(config, dict) or config.get('enabled') is not True:
        raise SuctionError('吸盘控制参数尚未确认')
    slave = config.get('slave_id')
    if type(slave) is not int or not 1 <= slave <= 15:
        raise SuctionError('吸盘站号必须为 1–15')
    for name in ('open_outputs', 'close_outputs'):
        outputs = config.get(name)
        if not isinstance(outputs, list) or not 1 <= len(outputs) <= 4:
            raise SuctionError('吸盘打开/关闭输出尚未配置')
        addresses = set()
        for output in outputs:
            if (not isinstance(output, dict) or set(output) != {'address', 'value'}
                    or type(output['address']) is not int or not 0 <= output['address'] <= 3
                    or type(output['value']) is not bool or output['address'] in addresses):
                raise SuctionError('吸盘输出必须是 O1–O4 的独立开关值')
            addresses.add(output['address'])
    if config['open_outputs'] == config['close_outputs']:
        raise SuctionError('吸盘打开与关闭不能使用相同指令')
    return config


def frame(payload):
    data = bytes(payload)
    crc = 0xffff
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xa001 if crc & 1 else crc >> 1
    return data + crc.to_bytes(2, 'little')


def valid_raw_request(args, sdk):
    """Broker boundary: permit only bounded relay writes and state reads."""
    if len(args) != 5:
        return False
    send_count, receive_count, data, output, ec = args
    if (type(send_count) is not int or send_count != 8
            or type(receive_count) is not int or receive_count not in (6, 8)
            or not isinstance(data, (list, tuple)) or len(data) != 8
            or any(type(v) is not int or not 0 <= v <= 255 for v in data)
            or not isinstance(output, sdk.PyTypeVectorInt) or output.content()
            or not isinstance(ec, dict)):
        return False
    data = bytes(data)
    if not 1 <= data[0] <= 15 or frame(data[:6]) != data:
        return False
    if data[1] == 1:
        return receive_count == 6 and data[2:6] == bytes([0, 0, 0, 8])
    return (receive_count == 8 and data[1] == 5 and data[2] == 0
            and 0 <= data[3] <= 3 and data[4] in (0, 255) and data[5] == 0)


def exchange(sdk, arm, payload, receive_count, audit=None):
    request = frame(payload)
    output = sdk.PyTypeVectorInt([])
    ec = {}
    invoke(audit, '左臂吸盘 RS485 报文', arm.XPRS485SendData,
           (len(request), receive_count, list(request), output), ec)
    if ec.get('ec', 0):
        raise SuctionError(f'吸盘指令未确认，输出状态未知: {ec}')
    values = output.content()
    if any(type(v) is not int or not 0 <= v <= 255 for v in values):
        raise SuctionError('吸盘回包字节异常，输出状态未确认')
    response = bytes(values)
    # AR v0.7.1 appends the response after receive_count zero placeholders,
    # even when given an empty output holder. Also accept an unpadded SDK reply.
    if len(response) == 2 * receive_count and response[:receive_count] == bytes(receive_count):
        response = response[receive_count:]
    if (len(response) != receive_count or response[0] != request[0]
            or response[1] != request[1] or frame(response[:-2]) != response):
        raise SuctionError('吸盘回包长度、地址、功能码或 CRC 异常，输出状态未确认')
    return request, response


def set_output(sdk, left_arm, config, enabled, audit=None):
    config = validate_config(config)
    if type(enabled) is not bool:
        raise SuctionError('吸盘开关必须为布尔值')
    # The typed coil API timed out on this AR. Use the verified raw-frame API;
    # never change 24 V supply or RS485 configuration, and never retry writes.
    outputs = config['open_outputs' if enabled else 'close_outputs']
    for output in outputs:
        request, reply = exchange(sdk, left_arm,
            [config['slave_id'], 5, 0, output['address'], 255 if output['value'] else 0, 0], 8, audit)
        if reply != request:
            raise SuctionError('吸盘指令回包与请求不一致，输出状态未确认')
    _, state = exchange(sdk, left_arm, [config['slave_id'], 1, 0, 0, 0, 8], 6, audit)
    if state[2] != 1 or any(bool(state[3] & (1 << o['address'])) != o['value'] for o in outputs):
        raise SuctionError('吸盘继电器回读状态与请求不一致，输出状态未确认')
    return {'commanded_open': enabled, 'confirmed': True}
