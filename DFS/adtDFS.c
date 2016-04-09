#include "adtDFS.h"

//
// IMPLEMENTASI HEADER
//

// variable GLOBAL
int found_end = 0;

void Start() {
	// masuk garis awal
	clearTimer(timer1);
	while (time1[timer1] <= 2000) {
		moveMotorTarget(leftMotor, 300, 100);
		moveMotorTarget(rightMotor, 300, 100);
	}
}

void getSimpang(Mem& m){
	// mencari simpangan yang ada

	//KAMUS
	string sr, sg, sb, gstr, count_str;
	long rv, gv, bv;
	long gyroDG;
	int counter = -1, limit = 0;

	//ALGORITMA

	//inisiasi nilai Mem
	m.arrDG[0] = -1;
	m.arrDG[1] = -1;
	m.arrDG[2] = -1;
	m.arrDG[3] = -1;
	m.currDG = -1;

	displayTextLine(11, "current procedure: getSimpang");

	forward(1500, milliseconds, 20);

	resetGyro(gyroSensor);
	gyroDG = getGyroDegrees(gyroSensor);

	//putar 360 derajat utk mencari banyak simpangan
	while (gyroDG <= 330) {

			motor[leftMotor] = 20;
			motor[rightMotor] = 0;

			getColorRGB(colorSensor, rv, gv, bv);
			stringFormat(gstr, "%.2d", gyroDG);
			stringFormat(sr, "%.2d", rv);
			stringFormat(sg, "%.2d", gv);
			stringFormat(sb, "%.2d", bv);

			displayTextLine(6, "gyro %s", gstr);
			displayTextLine(3, "red %s", sr);
			displayTextLine(4, "green %s", sg);
			displayTextLine(5, "blue %s", sb);
			//if (rv == 0 && gv == 0 && bv == 0) { //ketemu jalur hitam
			if (rv == 0 && (gv != 0 || gv == 0) && bv == 0) {
				if (limit == 0) {
					counter++;
					stringFormat(count_str, "%.2d", counter);
					displayTextLine(1, "jalur ke: %s", count_str);

					m.arrDG[counter] = gyroDG;
					stringFormat(gstr, "%.2d", 	m.arrDG[counter]);
					displayTextLine(7+counter, "%s: %s", count_str, gstr);
				}
				limit++;

			} else {
				limit = 0;
			}

		gyroDG = getGyroDegrees(gyroSensor);
	}

	motor[leftMotor] = 0;
	motor[rightMotor] = 0;

	for (int i = 0; i < 4; i++) {
		if ((m.arrDG[i] - 180 <= 30 && m.arrDG[i] >= 180) || (180 - m.arrDG[i] <= 30 && m.arrDG[i] <= 180)) {
			m.arrDG[i] = -1;
		}
	}
	wait1Msec(3000);
}

int gotonextline(Mem& m) {
	// memilih simpangan yang akan dilewati berikutnya
	// jika ada jalan yang belum dilewati, return 1
	// jika semua cabang sudah dilewati, return 0

	//KAMUS
	int found = -1;
	int buffcurrDG;
	long varcurrDG;
	long gyroDG;
	string currdg_str;

	//ALGORITMA
	eraseDisplay();
	displayTextLine(11, "current procedure: gotonextline");

	//cek apakah simpang ini baru dilalui pertama kali
	if (m.currDG == -1) {
		for (int i = 0; i < 4; i++) {
			if (m.arrDG[i] != -1) {
				//dapatkan sudut yang paling kecil
				found = i;
				m.currDG = m.arrDG[i];
				buffcurrDG = m.arrDG[i];
				m.arrDG[i] = -1;
				break;
			}
		}
	} else {
		buffcurrDG = m.currDG;
		for (int i = 0; i < 4; i++) {
			if (m.arrDG[i] != -1) {
				found = i;
				buffcurrDG = m.arrDG[i] - 180 - m.currDG;
				m.currDG = m.arrDG[i];
				//membuat hasil menjadi lebih presisi (pembulatan ke 0)
				if (buffcurrDG >= -20 && buffcurrDG <= 20) {
					buffcurrDG = 0;
				}
				m.arrDG[i] = -1;
				break;
			}
		}
	}

	//memutar robot sebesar currDG untuk berjalan di jalur yang bersangkutan
	if (found != -1) {
			stringFormat(currdg_str, "%.2d", buffcurrDG);
			displayTextLine(12, "currDG: %s", currdg_str);
			wait1Msec(2000);

			resetGyro(gyroSensor);
			gyroDG = getGyroDegrees(gyroSensor);

			if (buffcurrDG < 0) {
					varcurrDG = 360 + buffcurrDG;
			} else {
					varcurrDG = buffcurrDG;
			}

			stringFormat(currdg_str, "%.2d", varcurrDG);
			displayTextLine(12, "Sudut putar: %s", currdg_str);
			wait1Msec(1500);

			while (gyroDG <= varcurrDG + 40) {
					motor[leftMotor] = 20;
					motor[rightMotor] = 0;
					gyroDG = getGyroDegrees(gyroSensor);
			}

			displayTextLine(13, "currDG selesai");
			forward(500, milliseconds, 20);
			motor[leftMotor] = 0;
			motor[rightMotor] = 0;
			wait1Msec(1500);

			eraseDisplay();
	}

	return found;

}

void goback(){
		//bertemu jalan buntu
		//kembali ke node sebelumnya
		eraseDisplay();
		displayTextLine(12, "current procedure: goback");
		resetGyro(gyroSensor);
		while (getGyroDegrees(gyroSensor) <= 200) {
				motor[leftMotor] = 20;
				motor[rightMotor] = 0;
		}
		motor[leftMotor] = 0;
		motor[rightMotor] = 0;
		wait1Msec(1500);
}

int LineFollow(int &counterBlue){
/*
	line follower, jika hitam dan putih
	jika warna biru (Finish / Start) >> return 1
	jika warna hijau ( simpangan ) >> return 2
	jika warna merah ( Buntu) >> return 3
*/
		//KAMUS
		long redValue, greenValue, blueValue;
		string svaluer, svalueh, svalueb;

		//ALGORITMA

		while (true) {

			getColorRGB(colorSensor, redValue, greenValue, blueValue);
			stringFormat(svaluer, "%.2d", redValue);
			stringFormat(svalueh, "%.2d", greenValue);
			stringFormat(svalueb, "%.2d", blueValue);

			displayTextLine(3, "red intensity: %s", svaluer);
			displayTextLine(4, "green intensity: %s", svalueh);
			displayTextLine(5, "blue intensity: %s", svalueb);

			if (redValue == 0 && blueValue == 0 && greenValue != 0) {
					break;
			} else if (redValue != 0 && greenValue == 0 && blueValue == 0) {
					break;
			} else if (redValue == 0 && greenValue == 0 && blueValue != 0 && counterBlue == 0) {
					break;
			} else if (redValue == greenValue && redValue < 100 && blueValue >= 100 && counterBlue > 0) {
					break;
			} else { // line following (hitam/putih)
					if (getColorReflected(colorSensor) < 15) {
							motor[leftMotor] = 60;
							motor[rightMotor] = 15;
					}
					else {
							motor[leftMotor] = 15;
							motor[rightMotor] = 25;
					}
			}

		}

		// berhenti untuk mendeteksi warna selain hitam dan putih
		stopAllMotors();

		eraseDisplay();
		displayTextLine(2, "special color detected");
		displayTextLine(3, "red intensity: %s", svaluer);
		displayTextLine(4, "green intensity: %s", svalueh);
		displayTextLine(5, "blue intensity: %s", svalueb);

		if (redValue == 0 && blueValue == 0 && greenValue != 0) { //deteksi hijau
				displayTextLine(7, "Warna kotak: HIJAU");
				return 2;
		} else if (redValue != 0 && greenValue == 0 && blueValue == 0) { //deteksi merah
				displayTextLine(7, "Warna kotak: MERAH");
				return 3;
		} else if (redValue == 0 && greenValue == 0 && blueValue != 0 && counterBlue == 0) { //deteksi biru
				displayTextLine(7, "Warna kotak: BIRU-START");
				return 1;
		} else if (redValue == greenValue && blueValue > redValue && counterBlue > 0) { //deteksi biru
				displayTextLine(7, "Warna kotak: BIRU-FINISH");
				return 1;
		} else {
				displayTextLine(7, "Invalid color detected");
				turnLeft(150,milliseconds,20);
				turnRight(300,milliseconds,20);
				turnLeft(150,milliseconds,20);
				return 0;
		}

}

void gotofirst(Mem &m, int jn) {
	// prosedur untuk kembali ke kotak start setelah sampai di kotak finish

	//KAMUS
	int temp;
	long gyroDG;
	long putaran;
	int counterBlue = 1;
	string currdgstr;

	//ALGORITMA
	temp = LineFollow(counterBlue);
	wait1Msec(1500);

	while (temp != 0 && temp != 1) {
		if (temp == 2) {
				//menuju path yang disimpan pada currDG
				resetGyro(gyroSensor);
				gyroDG = getGyroDegrees(gyroSensor);
				eraseDisplay();
				forward(1000, milliseconds, 20);
				if (jn == 0) {
						putaran = 360 - m.currDG + 10;
				} else {
						putaran = m.currDG + 25;
				}
				stringFormat(currdgstr, "%.2d", putaran);
				displayTextLine(14, "Sudut putar: %s", currdgstr);
				while (gyroDG <= putaran) {
						motor[leftMotor] = 20;
						motor[rightMotor] = 0;
						gyroDG = getGyroDegrees(gyroSensor);
				}
				motor[leftMotor] = 0;
				motor[rightMotor] = 0;
				forward(1000, milliseconds, 20);
				wait1Msec(1500);
				break;
		}

	}

}

void gobacktrack(Mem &m) {
	//kembali ke simpangan sebelumnya

	//KAMUS
	long gyroDG;
	string currdgstr;

	//ALGORITMA
	eraseDisplay();
	displayTextLine(12, "current procedure: gobacktrack");
	resetGyro(gyroSensor);
	gyroDG = getGyroDegrees(gyroSensor);
	stringFormat(currdgstr, "%.2d", 360 - m.currDG + 20);
	displayTextLine(14, "Sudut putar: %s", currdgstr);
	wait1Msec(1500);

	while (gyroDG <= 360 - m.currDG + 20) {
			motor[leftMotor] = 20;
			motor[rightMotor] = 0;
			gyroDG = getGyroDegrees(gyroSensor);
	}

	motor[leftMotor] = 0;
	motor[rightMotor] = 0;
	forward(1000, milliseconds, 20);
	wait1Msec(1500);
}

void SolveDFS(){
	// prosedur mencari kotak finish dimulai dari kotak start

	//KAMUS
	int state;
	int counterBlue = 0;	// variabel penghitung banyak kotak biru yang ditemui
	Mem m[11];
	int idx=0; 						// Jumlah simpangan
	string intTostr;			// variabel konversi int to string
	int gtn;
	int TEMP_IDX;

	//ALGORITMA

	// inisiasi nilai Mem
	for (int i = 1; i <= 10; i++) {
		m[i].arrDG[0] = -1;
		m[i].arrDG[1] = -1;
		m[i].arrDG[2] = -1;
		m[i].arrDG[3] = -1;
		m[i].currDG = -1;
	}

	moveMotorTarget(leftMotor, 300, 100);
	moveMotorTarget(rightMotor, 300, 100);

	state = LineFollow(counterBlue); // mulai jalan dari start

	wait1Msec(1500);

	while (state != 0 && state != 1) { // selama tidak error dan belum finish

			if (state == 2) { // berada di persimpangan

					// sudah melewati kotak biru di START
					counterBlue = 1;

					// tambah jumlah simpangan
					idx++;
					eraseDisplay();
					stringFormat(intTostr, "%.2d", idx);
					displayTextLine(9, "Simpangan ke: %s", intTostr);

					// cari path apa saja yg dimiliki simpangan ke-idx jika baru pertama kali sampai di simpangan itu
					if (m[idx].currDG == -1) {
							getSimpang(m[idx]);
					} else {
							forward(1800, milliseconds, 20);
					}

					displayTextLine(1, "Simpangan ke-%s", intTostr);
					stringFormat(intTostr, "%.2d", m[idx].arrDG[0]);
					displayTextLine(1, "degree 0: %s", intTostr);
					stringFormat(intTostr, "%.2d", m[idx].arrDG[1]);
					displayTextLine(2, "degree 1: %s", intTostr);
					stringFormat(intTostr, "%.2d", m[idx].arrDG[2]);
					displayTextLine(3, "degree 2: %s", intTostr);
					stringFormat(intTostr, "%.2d", m[idx].arrDG[3]);
					displayTextLine(4, "degree 3: %s", intTostr);

					wait1Msec(1500);

					// mengecek apakah ada jalur yang belum dilewati
					gtn = gotonextline(m[idx]);
					if (gtn == -1) {
							eraseDisplay();
							stringFormat(intTostr, "%.2d", idx);
							displayTextLine(13, "Semua jalur di simpang ke-%s sudah dilewati", intTostr);
							wait1Msec(2000);
							gobacktrack(m[idx]);
							m[idx].arrDG[0] = -1;
							m[idx].arrDG[1] = -1;
							m[idx].arrDG[2] = -1;
							m[idx].arrDG[3] = -1;
							m[idx].currDG = -1;
							idx = idx - 2;
							if (idx == -1) {
									// solusi tidak ketemu
									break;
							}
							eraseDisplay();
					}

			} else { // berada di jalan buntu, balik
			  	goback();
			  	idx--;
			}

			state = LineFollow(counterBlue);

			if (state == 1) { //selesai, robot akan kembali ke kotak start dengan path yang sudah diingat
					eraseDisplay();
					displayTextLine(13, "Kembali ke start");
					wait1Msec(1500);
					goback();

				for (int i = idx; i >= 1; i--) {

						if (i == idx) {
								displayTextLine(3, "Jalur antara:");
								displayTextLine(4, "finish");
								stringFormat(intTostr, "%.2d", i);
								displayTextLine(5, "%s", intTostr);
						} else {
								displayTextLine(3, "Jalur antara:");
								stringFormat(intTostr, "%.2d", TEMP_IDX);
								displayTextLine(4, "%s", intTostr);
								stringFormat(intTostr, "%.2d", i);
								displayTextLine(5, "%s", intTostr);
						}
						wait1Msec(2000);
						TEMP_IDX = i;
						gotofirst(m[i], 0);

				}

				eraseDisplay();
				displayTextLine(3, "Jalur antara:");
				displayTextLine(4, "1");
				displayTextLine(5, "start");
				wait1Msec(2000);
				state = LineFollow(counterBlue);
				found_end = 1;

				eraseDisplay();
				displayTextLine(13, "Kembali ke finish lagi");
				wait1Msec(1500);
				goback();

				for (int i = 1; i <= idx; i++) {

						if (i == 1) {
								displayTextLine(3, "Jalur antara:");
								displayTextLine(4, "start");
								stringFormat(intTostr, "%.2d", i);
								displayTextLine(5, "%s", intTostr);
						} else {
								displayTextLine(3, "Jalur antara:");
								stringFormat(intTostr, "%.2d", TEMP_IDX);
								displayTextLine(4, "%s", intTostr);
								stringFormat(intTostr, "%.2d", i);
								displayTextLine(5, "%s", intTostr);
						}
						wait1Msec(2000);
						TEMP_IDX = i;
						gotofirst(m[i], 1);
				}

				eraseDisplay();
				displayTextLine(3, "Jalur antara:");
				stringFormat(intTostr, "%.2d", idx);
				displayTextLine(4, "%s", intTostr);
				displayTextLine(5, "finish");
				wait1Msec(2000);
				state = LineFollow(counterBlue);
				break;
		}

	}

}
