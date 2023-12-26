//
//  beatChecker.hpp
//  IK
//
//  Created by 최종원 on 2023/10/23.
//

#ifndef beatChecker_hpp
#define beatChecker_hpp
#include <fftw3.h>
#include "gnuplot-iostream.h"
#include "motion.hpp"
#define FRAME 30
#define PI 3.141592

namespace BC{

// 주어진 신호에 대한 FFT를 계산하는 함수
std::vector<std::complex<double>> calculateFFT(const std::vector<double>& signal) {
    int N = signal.size();
    fftw_complex* in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex* out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);

    for (int i = 0; i < N; i++) {
        in[i][0] = signal[i];
        in[i][1] = 0.0;  // 허수 부분은 0으로 설정
    }

    fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);

    std::vector<std::complex<double>> result(N);
    for (int i = 0; i < N; i++) {
        result[i] = std::complex<double>(out[i][0], out[i][1]);
    }

    fftw_free(in);
    fftw_free(out);

    return result;
}

// FFT 결과를 사용하여 역 FFT를 수행하여 신호를 재구성하는 함수
std::vector<double> calculateInverseFFT(const std::vector<std::complex<double>>& spectrum) {
    int N = spectrum.size();
    fftw_complex* in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);
    fftw_complex* out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * N);

    for (int i = 0; i < N; i++) {
        in[i][0] = real(spectrum[i]);
        in[i][1] = imag(spectrum[i]);
    }

    fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);

    std::vector<double> result(N);
    for (int i = 0; i < N; i++) {
        result[i] = out[i][0] / N;  // 스케일링
    }

    fftw_free(in);
    fftw_free(out);

    return result;
}


void beatCheck(Motion &m, int w){
    vector<int> qPeak;
    vector<int> vBottom;
//    vector<vector<int>> candidates;
    vector<vector<double>> sinosoid;
    vector<double> momentum;
    vector<double> total;
//    vector<vector<double>> ve;
    vector<double> energy;
    
    for(int i = 0; i < m.bodies.size()-1; i++) {
        for(int j = 0; j < m.bodies[i].links.size(); j++) {
            if(m.bodies[i].links[j].isHand || m.bodies[i].links[j].isEnd) continue;
            m.bodies[i].links[j].v = (m.bodies[i+1].links[j].getPos() - m.bodies[i].links[j].getPos());
        }
    }
    
    for(int i = 0; i < m.bodies[0].links.size(); i++) {
        float energy = 0;
        for(int j = 0; j < m.bodies.size(); j++) {
            energy += length(m.bodies[j].links[i].v);
        }
        momentum.push_back(energy/m.bodies.size());
    }
    
    for(int i = 0; i < m.bodies.size(); i++) {
        double e = 0;
        for(int j = 0; j < m.bodies[0].links.size(); j++) {
            e += length(m.bodies[i].links[j].v);
        }
        energy.push_back(e);
    }
    
    
    float momentumSum = 0;
    for(auto m : momentum){
        momentumSum += m;
    }
//    std::cout << momentumSum<< std::endl;
    vector<double> beats;
    int firstBeatF = 0;
//    float energyEver;
//    for(auto &e : energy) energyEver += e/energy.size();
    for(int i = w; i < m.bodies.size()-w; i++) {
        bool beat = true;
        float thresholdL = 0;
        float thresholdR = 0;
        float threshold = 0;
        for(int j = -w; j <= w; j++) {
            threshold += energy[i+j] - energy[i];
        }
        for(int j = -w; j <= w; j++) {
            if(energy[i] > energy[i+j]) beat = false;
        }
//        for(int j = -w; j < 0; j++) {
//            thresholdL += energy[i+j] - energy[i];
//        }
//        for(int j = 0; j <= w; j++) {
//            thresholdR += energy[i+j] - energy[i];
//        }
//        if(energy[i] > momentumSum * 1.3f || thresholdL < 2*momentumSum || thresholdR < 2*momentumSum){
        if(energy[i] > momentumSum * 1.3f || threshold < 2*momentumSum){
            beat = false;
        }
        if(beat){
            for(int j = 0; j < i; j++){
                if(energy[j] > momentumSum){
                    m.bodies[i].isBeat = true;
                    beats.push_back(i);
                    break;
                }
            }
        }
        
        if(firstBeatF == 0 && m.bodies[i].isBeat) firstBeatF = i;
    }
    
    
    if(beats.size() == 0) return;
    
    vector<double> beatSinosoid;

    for(int i = 0; i < beats[0]; i++) beatSinosoid.push_back(0);
    for(int i = 0; i < beats.size()-1; i++) {
        for(int j = 0; j < beats[i+1]-beats[i]; j++) {
            beatSinosoid.push_back( glm::cos(2*PI*(j / double(beats[i+1]-beats[i]))) );
        }
    }
    for(int i = beats[beats.size()-1]; i < m.bodies.size(); i++) beatSinosoid.push_back(0);

    std::vector<std::complex<double>> fftResult = calculateFFT(beatSinosoid);

    vector<double> f;
    float mainF = 0;
    float maxV = 0;
    float sumF = 0;
    float maxF = 0;

    for(int i = 0; i < fftResult.size()/2; i++){
        float amplitude = sqrt(fftResult[i].real()*fftResult[i].real() + fftResult[i].imag()*fftResult[i].imag())/fftResult.size();
        if(fftResult.size()/(i+1) < 10 || fftResult.size()/(i+1) > 60) amplitude = 0;
        f.push_back(amplitude);
        
        if(amplitude > maxV){
            maxV = amplitude;
            maxF = i;
        }
        if(amplitude > 0.1f){
            sumF += amplitude;
        }
    }
    
    mainF = maxF;
        
    mainF = round(fftResult.size()/mainF);

//    std::cout << mainF << std::endl;
//    std::cout << std::endl;
    
    vector<int> meterCap;
    for(int i = 0; i < beats.size()-1; i++){
        if(beats[i+1] - beats[i] < mainF*1.2f && beats[i+1] - beats[i] > mainF*0.8f) {
//            if(meterCap.size() == 0 && energy[i] > momentumSum/2) continue;
            meterCap.push_back(beats[i]);
            if(meterCap.size() == 3){
                meterCap.push_back(beats[i+1]);
                i++;
                m.bodies[meterCap[0]].isMeter = true;
                m.bodies[meterCap[0]].isStart = true;
                m.bodies[meterCap[0]].endMeter = meterCap[3];
                m.bodies[meterCap[3]].isMeter = true;
                m.bodies[meterCap[3]].isEnd = true;
                meterCap.clear();
            }
        }
        else meterCap.clear();
    }
    
//    Gnuplot gp("/usr/local/Cellar/gnuplot/5.4.10/bin/gnuplot");
////    gp << "set xrange [0:"<< to_string(m.bodies.size())<< "]\nset yrange [:]\n";
//////    gp << "set xrange [0:70]\nset yrange [:]\n";
//////    gp << "plot '-' with lines title '1','-' with lines title '2','-' with lines title '3'\n";
////    gp << "set xrange [0:"<< to_string(m.bodies.size()/2)<< "]\nset yrange [0:]\n";
//    gp << "set xlabel '주파수'\n";
//    gp << "plot '-' with lines title '속도의 크기','-' with lines title '비트 정현파'\n";
////    gp << "plot '-' with lines title 'PSD'\n";
////
////    gp.send1d(f);
//////    gp.send1d(total);
////
////    gp.send1d(cc2);
//    gp.send1d(energy);
//    gp.send1d(beatSinosoid);
//////    gp.send1d(momentum);
//    getchar();
    
}
}
#endif /* beatChecker_hpp */
