// IMU transaction/setup/sample routines copied from the qualified LQR driver.
static unsigned i2c_read(unsigned index){return rd(index*4)&0xffu;}
static void i2c_write(unsigned index,unsigned value){wr(index*4,value);}
static uint64_t now_ns(){return static_cast<uint64_t>(now_seconds()*1e9);}
static bool i2c_command(unsigned command, bool check_ack)
{
    i2c_write(4, command | 0x01); // IACK.
    const uint64_t deadline = now_ns() + 20000000u;
    do {
        const unsigned status = i2c_read(4);
        if (status & 0x20u) { fprintf(stderr, "I2C: arbitraggio perso\n"); return false; }
        if ((status & 0x01u) && !(status & 0x02u)) {
            if (check_ack && (status & 0x80u)) {
                fprintf(stderr, "I2C: NACK\n"); return false;
            }
            return true;
        }
        usleep(50);
    } while (!stop_requested && now_ns() < deadline);
    fprintf(stderr, "I2C: timeout/interruzione\n");
    return false;
}

static bool bus_free()
{
    const uint64_t deadline = now_ns() + 20000000u;
    do {
        if (!(i2c_read(4) & 0x40u)) return true;
        usleep(50);
    } while (!stop_requested && now_ns() < deadline);
    fprintf(stderr, "I2C: bus occupato\n");
    return false;
}

static bool imu_read(unsigned address, unsigned char *data, unsigned length)
{
    i2c_write(3, 0x68u << 1);
    if (!i2c_command(0x90, true)) return false;
    i2c_write(3, address);
    if (!i2c_command(0x10, true)) return false;
    i2c_write(3, (0x68u << 1) | 1u);
    if (!i2c_command(0x90, true)) return false;
    for (unsigned i = 0; i < length; ++i) {
        if (!i2c_command(i + 1u == length ? 0x68 : 0x20, false)) return false;
        data[i] = static_cast<unsigned char>(i2c_read(3));
    }
    return bus_free();
}

static bool imu_write(unsigned address, unsigned value)
{
    i2c_write(3, 0x68u << 1);
    if (!i2c_command(0x90, true)) return false;
    i2c_write(3, address);
    if (!i2c_command(0x10, true)) return false;
    i2c_write(3, value);
    return i2c_command(0x50, true) && bus_free();
}

static bool imu_setup()
{
    i2c_write(2, 0);
    i2c_write(4, 1);
    i2c_write(0, 99); // 100 kHz da clock FPGA di 50 MHz.
    i2c_write(1, 0);
    i2c_write(2, 0x80);
    unsigned char id = 0;
    if (!bus_free() || !imu_read(0x75, &id, 1) || id != 0x70u) {
        fprintf(stderr, "MPU-6500 WHO_AM_I inatteso: 0x%02x\n", id);
        return false;
    }
    if (!imu_write(0x6B, 0x80)) return false;
    usleep(100000);
    const unsigned addresses[] = {0x6B, 0x6C, 0x1B, 0x1C, 0x1A, 0x1D, 0x19};
    const unsigned values[] = {0x01, 0x00, 0x00, 0x00, 0x03, 0x03, 0x09};
    for (unsigned i = 0; i < 7; ++i)
        if (!imu_write(addresses[i], values[i])) return false;
    usleep(100000);
    for (unsigned i = 0; i < 7; ++i) {
        unsigned char value = 0;
        if (!imu_read(addresses[i], &value, 1) || value != values[i]) {
            fprintf(stderr, "Configurazione IMU non confermata: registro 0x%02x\n",
                    addresses[i]);
            return false;
        }
    }
    return true;
}

static int signed_word(const unsigned char *p)
{
    int value = (static_cast<int>(p[0]) << 8) | p[1];
    return value >= 32768 ? value - 65536 : value;
}

struct Sample {
    int ax, ay, az, gx, gy, gz;
    double angle_deg, gyro_deg_s, accel_g;
};

static bool get_sample(Sample *sample)
{
    unsigned char data[14];
    if (!imu_read(0x3B, data, 14)) return false;
    sample->ax = signed_word(data);
    sample->ay = signed_word(data + 2);
    sample->az = signed_word(data + 4);
    sample->gx = signed_word(data + 8);
    sample->gy = signed_word(data + 10);
    sample->gz = signed_word(data + 12);
    const double x = static_cast<double>(sample->ax);
    const double y = static_cast<double>(sample->ay);
    const double z = static_cast<double>(sample->az);
    sample->accel_g = std::sqrt(x*x + y*y + z*z) / 16384.0;
    // Con il robot verticale sul telaio attuale l'IMU misura az circa +1 g.
    sample->angle_deg = std::atan2(-x, z) * (180.0 / 3.141592653589793);
    sample->gyro_deg_s = sample->gy / 131.0;
    return true;
}

