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

template<typename LowGData>
void output(LowGData d, uint32_t timestamp){

}

template<uint8_t I>
void read(uint8_t idx, std::ifstream& file){
    if constexpr(I >= std::tuple_size_v<LoggedReadingType>) {
        std::cout << "invlaid discriminant " << idx << '\n';
        exit(1);
    } else {
        if(I == idx){
            using Type = typename std::tuple_element<I, LoggedReadingType>::type;
            
            Type data;
            uint32_t timestamp;
            file.read((char*)&timestamp, sizeof(timestamp));
            file.read((char*)&data, sizeof(data));
            output(data, timestamp);
        
        } else {
            return read<I + 1>(idx, file);
        }
    }
}

int main(int argc, char** argv) {
    if(argc != 2){
        std::cout << "Usage log_parser.exe [log_path]\n";
        return 1;
    }

    std::ifstream file {argv[1]};
    if(!file){
        std::cout << "Filed to open " << argv[1] << '\n';
        return 1;
    }

    uint32_t checksum;
    CHECK(file.read((char*)&checksum, 4));

    while(true){
        ReadingDiscriminant d;
        file.read((char*)&d, sizeof(d));
        if(!file) break;

        if(d < 1 || d >= READING_DISCRIMINANT_SIZE){
            std::cout << "invalid discriminant " << d << "\n";
            return 1;
        }

        read<1>(d, file);
    }

    std::cout << "done\n";
}