#main cppp
#include <chrono>
#include <thread>
#include <unitree/idl/hg/HandState_.hpp> //replace your sdk path
#include <unitree/idl/hg/HandCmd_.hpp> //replace your sdk path
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <mutex>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go1/LowState_.hpp>
#include <unitree/idl/hg/HandState_.hpp>

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <cmath>
#include <jsoncpp/json/json.h>
#include <thread>
#include <chrono>

using namespace unitree::robot;
using namespace std;
using namespace std::chrono;

vector<int> BRAZO_IZQ  = {15,16,17,18,19,20,21};
vector<int> BRAZO_DER  = {22,23,24,25,26,27,28};
vector<int> CINTURA    = {12,13,14};

shared_ptr<ChannelSubscriber<unitree_go::msg::dds_::LowState_>> lowstate_sub;
shared_ptr<ChannelSubscriber<unitree_hg::msg::dds_::HandState_>> handleft_sub;
shared_ptr<ChannelSubscriber<unitree_hg::msg::dds_::HandState_>> handright_sub;

unitree_go::msg::dds_::LowState_ lowstate;
unitree_hg::msg::dds_::HandState_ handleft;
unitree_hg::msg::dds_::HandState_ handright;

bool low_ready = false, hand_ready = false;

void lowstate_callback(const void *msg) {
    lowstate = *(unitree_go::msg::dds_::LowState_ *)msg;
    low_ready = true;
}

void handleft_callback(const void *msg) {
    handleft = *(unitree_hg::msg::dds_::HandState_ *)msg;
    hand_ready = true;
}

void handright_callback(const void *msg) {
    handright = *(unitree_hg::msg::dds_::HandState_ *)msg;
    hand_ready = true;
}

map<string, float> leer_posiciones(const vector<int>& ids) {
    map<string, float> posiciones;
    for (int id : ids) {
        posiciones[to_string(id)] = lowstate.motor_state()[id].q();
    }
    return posiciones;
}

map<string, float> leer_mano(bool izquierda) {
    map<string, float> posiciones;
    auto& estado = izquierda ? handleft : handright;
    string prefijo = izquierda ? "mano_izq_" : "mano_der_";
    for (int i = 0; i < estado.motor_state().size(); ++i) {
        posiciones[prefijo + to_string(i)] = estado.motor_state()[i].q();
    }
    return posiciones;
}

void mostrar_posiciones(const map<string, float>& posiciones) {
    for (const auto& [id, valor] : posiciones) {
        cout << id << ": " << valor << " rad (" << valor * 180.0 / M_PI << " deg)" << endl;
    }
}

void guardar_rutina(const vector<Json::Value>& pasos) {
    string nombre;
    cout << "Nombre de la rutina: ";
    getline(cin, nombre);
    if (nombre.empty()) nombre = "rutina_sin_nombre";

    Json::Value root;
    root["nombre_rutina"] = nombre;
    root["numero_pasos"] = static_cast<int>(pasos.size());
    root["pasos"] = Json::arrayValue;
    for (const auto& p : pasos)
        root["pasos"].append(p);

    ofstream file(nombre + ".json");
    file << root.toStyledString();
    file.close();

    cout << "Rutina guardada en " << nombre << ".json\n";
}

int main(int argc, char **argv) {
    if (argc < 2) {
        cout << "Uso: ./captura <interfaz_red>" << endl;
        return 1;
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    lowstate_sub = make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
    lowstate_sub->InitChannel(lowstate_callback, 10);

    handleft_sub = make_shared<ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>("rt/dex3/left/state");
    handleft_sub->InitChannel(handleft_callback, 10);

    handright_sub = make_shared<ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>("rt/dex3/right/state");
    handright_sub->InitChannel(handright_callback, 10);

    cout << "Esperando conexión..." << endl;
    while (!low_ready || !hand_ready) std::this_thread::sleep_for(milliseconds(100));
    cout << "Conectado.\n";

    vector<Json::Value> pasos;
    int contador = 1;

    while (true) {
        cout << "\nCapturando paso " << contador << ". Enter para continuar o 'q' para salir: ";
        string entrada;
        getline(cin, entrada);
        if (entrada == "q") break;

        auto brazo = leer_posiciones(BRAZO_IZQ);
        auto brazo_d = leer_posiciones(BRAZO_DER);
        auto cintura = leer_posiciones(CINTURA);
        auto mano_izq = leer_mano(true);
        auto mano_der = leer_mano(false);

        Json::Value paso;
        paso["nombre"] = "Paso " + to_string(contador);
        paso["posiciones"] = Json::objectValue;

        for (const auto& [id, q] : brazo)     paso["posiciones"][id] = q;
        for (const auto& [id, q] : brazo_d)   paso["posiciones"][id] = q;
        for (const auto& [id, q] : cintura)   paso["posiciones"][id] = q;
        for (const auto& [id, q] : mano_izq)  paso["posiciones"][id] = q;
        for (const auto& [id, q] : mano_der)  paso["posiciones"][id] = q;

        cout << "Duración en segundos para este paso: ";
        float dur;
        cin >> dur;
        cin.ignore(); // limpiar buffer
        paso["duracion"] = dur;

        pasos.push_back(paso);
        contador++;
    }

    guardar_rutina(pasos);
    return 0;
}
