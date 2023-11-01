#include<fstream>
#include<iostream>
#include"../../MIDAS/src/log_format.h"
#include<type_traits>

#define CHECK(x) if(!(x)) {std::cout << #x << " | (line " << __LINE__ << ") failed\n"; exit(1);}

static const char* id_to_name[] = {
    /* 0 */                   "ERROR",
    /*[ID_LOWG]*/             "ID_LOWG",
    /*[ID_HIGHG]*/            "ID_HIGHG",
    /*[ID_BAROMETER]*/        "ID_BAROMETER",
    /*[ID_CONTINUITY]*/       "ID_CONTINUITY",
    /*[ID_VOLTAGE]*/          "ID_VOLTAGE",
    /*[ID_GPS]*/              "ID_GPS",
    /*[ID_MAGNETOMETER]*/     "ID_MAGNETOMETER",
    /*[ID_ORIENTATION]*/      "ID_ORIENTATION",
    /*[ID_LOWGLSM]*/          "ID_LOWGLSM",
};

template<typename T>
void format(T t){
    if constexpr (std::is_same_v<uint8_t, T> || std::is_same_v<int8_t, T>){
        std::cout << (int) t;
    } else if constexpr (std::is_integral_v<T> || std::is_floating_point_v<T>){
        std::cout << t;
    } else if constexpr (std::is_same_v<const char *, T>){
        std::cout << '"' << t << '"';
    } else {
        static_assert(std::is_same_v<void, T>);
    }
}

#define JSON_FIELD(name,val) do{std::cout << '"' << #name << "\":"; format(val); std::cout << ',';} while(0)
#define JSON_FIELD_NC(name,val) do{std::cout << '"' << #name << "\":"; format(val);} while(0)
#define FIELD(name) JSON_FIELD(name, d.name)
#define FIELD_NC(name) JSON_FIELD_NC(name, d.name)

template<typename T> 
void generic_output(T d, uint32_t timestamp, ReadingDiscriminant discriminant){
    std::cout << '{';
    JSON_FIELD(sensor, id_to_name[discriminant]);
    JSON_FIELD(id, (int)discriminant);
    JSON_FIELD(timestamp, timestamp);
    std::cout << "\"data\":{";
    output(d, timestamp, discriminant);
    std::cout << "}}";
}

void output(LowGData d, uint32_t timestamp, ReadingDiscriminant discriminant){
    FIELD(gx); FIELD(gy); FIELD_NC(gz);
}

void output(HighGData d, uint32_t timestamp, ReadingDiscriminant){
    FIELD(gx); FIELD(gy); FIELD_NC(gz);
}

void output(Barometer d, uint32_t timestamp, ReadingDiscriminant){
    FIELD(temperature); FIELD_NC(pressure);
}

void output(Continuity d, uint32_t timestamp, ReadingDiscriminant){
    FIELD_NC(is_continuous);
}

void output(Voltage d, uint32_t timestamp, ReadingDiscriminant){
    FIELD_NC(voltage);
}

void output(GPS d, uint32_t timestamp, ReadingDiscriminant){
    FIELD(latitude); FIELD(longitudinal); FIELD(altitude); FIELD_NC(satellite_count);
}

void output(Magnetometer d, uint32_t timestamp, ReadingDiscriminant){
    FIELD(mx); FIELD(my); FIELD_NC(mz);
}

void output(Orientation d, uint32_t timestamp, ReadingDiscriminant){
    FIELD(yaw); FIELD(pitch); FIELD(roll);
    FIELD(orientation_velocity.vx);
    FIELD(orientation_velocity.vy);
    FIELD(orientation_velocity.vz);
    FIELD(orientation_acceleration.ax);
    FIELD(orientation_acceleration.ay);
    FIELD(orientation_acceleration.az);
    FIELD(gx); FIELD(gy); FIELD(gz);
    FIELD(magnetometer.mx);
    FIELD(magnetometer.my);
    FIELD(magnetometer.mz);
    FIELD_NC(temperature);

}

void output(LowGLSM d, uint32_t timestamp, ReadingDiscriminant){
    FIELD(gx); FIELD(gy); FIELD(gz);
    FIELD(ax); FIELD(ay); FIELD_NC(az);
}


template<uint8_t I>
void read(uint8_t idx, std::ifstream& file){
    if constexpr(I > std::tuple_size_v<LoggedReadingType>) {
        std::cerr << "invlaid discriminant2 " << (int)idx << '\n';
        exit(1);
    } else {
        if(I == idx){
            using Type = typename std::tuple_element<I-1, LoggedReadingType>::type;
            
            Type data;
            uint32_t timestamp;
            file.read((char*)&timestamp, sizeof(timestamp));
            file.read((char*)&data, sizeof(data));
            generic_output(data, timestamp, (ReadingDiscriminant)I);
        
        } else {
            return read<I + 1>(idx, file);
        }
    }
}

int main(int argc, char** argv) {
    if(argc != 2){
        std::cerr << "Usage log_parser.exe [log_path]\n";
        return 1;
    }

    std::ifstream file {argv[1], std::ios::binary};
    if(!file){
        std::cerr << "Filed to open " << argv[1] << '\n';
        return 1;
    }

    uint32_t checksum;
    CHECK(file.read((char*)&checksum, 4));

    std::cout << "{\"readings\": [\n";
    for(int row = 0;; row++) {
        ReadingDiscriminant d;
        file.read((char*)&d, sizeof(d));
        if(file.eof()) break;

        if (row != 0){
            std::cout << ",\n";
        }

        if(d < 1 || d >= READING_DISCRIMINANT_SIZE){
            std::cerr << "invalid discriminant [" << (int)d << "]\n";
            return 1;
        }

        read<1>(d, file);
    }

    std::cout << "\n]}";
}
