import numpy as np
from pydub import AudioSegment
import sys

def convert_mp3_to_c_array(mp3_file, array_name):
    # 读取 MP3 文件并转换为 PCM
    audio = AudioSegment.from_mp3(mp3_file)
    
    # 将音频数据转换为 16 位 PCM 格式
    audio = audio.set_frame_rate(44100).set_channels(1).set_sample_width(2)  # 单声道，16 位
    pcm_data = np.array(audio.get_array_of_samples())

    # 将数据转换为 C 代码格式
    hex_array = ', '.join(f'0x{b & 0xFFFF:04x}' for b in pcm_data)

    # 生成 C 代码
    c_code = f'const int16_t {array_name}[] = {{ {hex_array} }};\n'
    c_code += f'const size_t {array_name}_size = sizeof({array_name}) / sizeof({array_name}[0]);\n'

    return c_code

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python convert_mp3_to_c_array.py <input.mp3> <array_name>")
        sys.exit(1)

    mp3_file = sys.argv[1]
    array_name = sys.argv[2]

    c_array_code = convert_mp3_to_c_array(mp3_file, array_name)
    print(c_array_code)