#pragma config(Sensor, S1,     touchSensor,    sensorEV3_Touch)
#pragma config(Sensor, S2,     gyroSensor,     sensorEV3_Gyro, modeEV3Gyro_RateAndAngle)
#pragma config(Sensor, S3,     colorSensor,    sensorEV3_Color, modeEV3Color_Color)
#pragma config(Motor,  motorA,          armMotor,      tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorB,          leftMotor,     tmotorEV3_Large, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          rightMotor,    tmotorEV3_Large, PIDControl, driveRight, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "adtBFS.h"

// variable GLOBAL
Mem fixm[11];
int found_end = 0;
int tail = 1;
int banyaknextline = 1;
int countnextline;
int gtn;
int BACK_FOUND = 0;
int BACK_BLUE = 0;
int WHO_IS_NULL;
int TEMP_IDX;
int DEEP_COUNT;
string intTostr;		// variabel konversi int to string

void Start() {
		// prosedur untuk masuk garis awal
		clearTimer(timer1);
		while (time1[timer1] <= 1500) {
				moveMotorTarget(leftMotor, 300, 100);
				moveMotorTarget(rightMotor, 300, 100);
		}
}

void getSimpang(Mem& m){
		//mencari simpangan yang ada

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

		eraseDisplay();
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
				displayTextLine(12, "m.currDG != -1");
				wait1Msec(1500);
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

				wait1Msec(1500);

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
				motor[leftMotor] = 0;
				motor[rightMotor] = 0;
				forward(500, milliseconds, 20);
				wait1Msec(2000);

				eraseDisplay();
		}

		return found;

}

int LineFollow(int &counterBlue){
// line follower, jika hitam dan putih
// jika warna biru (Finish / Start) >> return 1
// jika warna hijau ( simpangan ) >> return 2
// jika warna merah ( Buntu) >> return 3

		long redValue, greenValue, blueValue;
		string svaluer, svalueh, svalueb;

		eraseDisplay();

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

void gotofirst(Mem &m, int jn, int &isBlue) {
		// prosedur untuk kembali ke kotak start setelah sampai di kotak finish

		//KAMUS
		int temp;
		long gyroDG;
		long putaran;
		int counterBlue = 1;
		string intTostr;

		//ALGORITMA
		temp = LineFollow(counterBlue);
		wait1Msec(1500);
		eraseDisplay();
		if (temp == 1) {
				isBlue = 1;
		}
		while (temp != 0 && temp != 1) {
				if (temp == 2) {
						//menuju path yang disimpan pada currDG
						resetGyro(gyroSensor);
						gyroDG = getGyroDegrees(gyroSensor);
						forward(1000, milliseconds, 20);
						if (jn == 0) {
								putaran = 360 - m.currDG + 10;
						} else {
								putaran = m.currDG + 25;
						}
						stringFormat(intTostr, "%.2d", putaran);
						displayTextLine(14, "Sudut putar: %s", intTostr);
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

void goback() {
		//bertemu jalan buntu
		//kembali ke node sebelumnya
		eraseDisplay();
		displayTextLine(12, "current procedure: goback");
		resetGyro(gyroSensor);
		while (getGyroDegrees(gyroSensor) <= 215) {
				motor[leftMotor] = 20;
				motor[rightMotor] = 0;
		}
		motor[leftMotor] = 0;
		motor[rightMotor] = 0;
		wait1Msec(1500);
}

void SolveBFS(){
	// prosedur mencari kotak finish dimulai dari kotak start

	//KAMUS
	int state;
	int counterBlue = 0;	// variabel penghitung banyak kotak biru yang ditemui
	Mem m[11];
	int idx, fix_idx; // Jumlah simpangan

	//ALGORITMA

	// inisiasi jumlah simpangan awal
	idx = 0;

	// inisiasi nilai Mem
	for (int i = 1; i <= 10; i++) {
			m[i].indexJalur = -1;
			m[i].friend[0] = -1;
			m[i].friend[1] = -1;
			m[i].friend[2] = -1;
			m[i].friend[3] = -1;
			m[i].connectedWith = -1;
			m[i].arrDG[0] = -1;
			fixm[i].arrDG[0] = -1;
			m[i].arrDG[1] = -1;
			fixm[i].arrDG[1] = -1;
			m[i].arrDG[2] = -1;
			fixm[i].arrDG[2] = -1;
			m[i].arrDG[3] = -1;
			fixm[i].arrDG[3] = -1;
			m[i].currDG = -1;
			fixm[i].currDG = -1;
	}

	moveMotorTarget(leftMotor, 300, 100);
	moveMotorTarget(rightMotor, 300, 100);

	state = LineFollow(counterBlue); // mulai jalan dari start
	wait1Msec(1500);
	eraseDisplay();

	while (state != 0 && state != 1) {
			// selama tidak error dan belum finish

			if (state == 2) { //detect Hijau

					// sudah melewati kotak biru di START
					counterBlue = 1;

					// tambah jumlah simpangan
					idx++;
					m[idx].connectedWith = 0;		//simpang ke-1 berhubungan dengan kotak start
					stringFormat(intTostr, "%.2d", idx);
					displayTextLine(9, "Simpangan ke: %s", intTostr);

					// cari path apa saja yg dimiliki simpangan
					if (m[idx].currDG == -1) {
							//membuat child node untuk simpang pertama (root)
							getSimpang(m[idx]);

							fixm[idx].arrDG[0] = m[idx].arrDG[0];
							fixm[idx].arrDG[1] = m[idx].arrDG[1];
							fixm[idx].arrDG[2] = m[idx].arrDG[2];
							fixm[idx].arrDG[3] = m[idx].arrDG[3];
							fixm[idx].currDG = m[idx].currDG;

					}

					eraseDisplay();
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
					eraseDisplay();

					countnextline = 1;
					DEEP_COUNT = 0;
					fix_idx = idx;
					while (countnextline <= banyaknextline) {
							gtn = gotonextline(m[idx]);		//pilih jalur dengan sudut terkecil

							if (gtn != -1) {

									m[idx].indexJalur = gtn;
									state = LineFollow(counterBlue);		//jalan menurut jalur yang dipilih pada gtn

									if (countnextline == banyaknextline) {
											if (state == 2) {
													DEEP_COUNT++;
													fix_idx++;
													tail++;												//tambahan
													m[idx].friend[gtn] = tail;		//tambahan
													m[tail].connectedWith = idx;	//tambahan

													eraseDisplay();
													stringFormat(intTostr, "%.2d", fix_idx);
													displayTextLine(9, "Simpangan ke: %s", intTostr);

													//membuat child node untuk simpang ke-tail
													getSimpang(m[tail]);	//tambahan

													fixm[tail].arrDG[0] = m[tail].arrDG[0];
													fixm[tail].arrDG[1] = m[tail].arrDG[1];
													fixm[tail].arrDG[2] = m[tail].arrDG[2];
													fixm[tail].arrDG[3] = m[tail].arrDG[3];
													fixm[tail].currDG = m[tail].currDG;

													eraseDisplay();
													displayTextLine(14, "balik ke parent");
													goback();
													state = LineFollow(counterBlue);

													//sampai di parent, maju sedikit
													forward(1800, milliseconds, 20);

											} else if (state == 1) {
													// kotak finish ditemukan, siap2 kembali ke start
													found_end = 1;
													break;
											} else if (state == 0) {
													displayTextLine(13, "ERROR state 0");
													wait1Msec(2000);
											} else {
													//state = 3 //tidak akan membuat child node
													fixm[idx].arrDG[gtn] = -1;

													eraseDisplay();
													stringFormat(intTostr, "%.2d", idx);
													displayTextLine(12, "Balik ke simpangan: %s", intTostr);
													wait1Msec(1500);
													goback();
													state = LineFollow(counterBlue);
													wait1Msec(1000);
													forward(1800, milliseconds, 20);
													wait1Msec(1000);
											}
									} else { //countnextline != banyaknextline
											if (state == 2) {
													idx = m[idx].friend[gtn];	//tambahan

													eraseDisplay();
													stringFormat(intTostr, "%.2d", idx);
													displayTextLine(9, "Simpangan ke: %s", intTostr);

													//tambahan X
													forward(1500, milliseconds, 20);

													fix_idx = idx;
													countnextline++;
											} else if (state == 1) {
													// kotak finish ditemukan, siap2 kembali ke start
													found_end = 1;
													break;
											} else if (state == 0) {
													displayTextLine(13, "ERROR state 0");
													wait1Msec(3000);
											} else {
													//state = 3
													eraseDisplay();
													stringFormat(intTostr, "%.2d", idx);
													displayTextLine(12, "Balik ke simpangan: %s", intTostr);
													wait1Msec(1500);
													goback();
													state = LineFollow(counterBlue);
													forward(1800, milliseconds, 20);
											}

									}
							} else { //gtn = -1
									eraseDisplay();
									displayTextLine(13, "Semua jalur sudah dicek");
									wait1Msec(1500);

									int found = 0;

									while (idx >= 1 && found == 0) {
											WHO_IS_NULL = idx;
											stringFormat(intTostr, "%.2d", idx);
											displayTextLine(13, "Simpangan ke: %s", intTostr);

											long gyroDG;
											long vargyroDG;
											//arahkan robot ke posisi awal saat di root
											resetGyro(gyroSensor);
											gyroDG = getGyroDegrees(gyroSensor);
											if (idx == 1) {
													vargyroDG = 180 + (360 - m[idx].currDG) + 20;
											} else {
													vargyroDG =  720 - m[idx].currDG + 20;
											}

											while (gyroDG <= vargyroDG) {
													motor[leftMotor] = 20;
													motor[rightMotor] = 0;
													gyroDG = getGyroDegrees(gyroSensor);
											}
											motor[leftMotor] = 0;
											motor[rightMotor] = 0;
											wait1Msec(1500);

											if (idx > 1) {
													//jalan sampai ketemu parent
													countnextline--;
													state = LineFollow(counterBlue);
													forward(1800, milliseconds, 20);
													wait1Msec(1500);

													idx = m[idx].connectedWith;
													found = 0;
													for (int i = 0; i < 4; i++) {
															if (m[idx].arrDG[i] != -1) {
																	found = 1;
																	break;
															}
													}
												} else {
														break;
												}
									} //endwhile backtrack
										//ulangi proses yang sama

									if (WHO_IS_NULL == 1) {

											if (DEEP_COUNT == 0) {
													break;
											} else {
													DEEP_COUNT = 0;
													banyaknextline++;
													eraseDisplay();
													stringFormat(intTostr, "%.2d", banyaknextline);
													displayTextLine(13, "Cek sampai level: %s", intTostr);
													wait1Msec(1500);
													countnextline = 1;
													idx = 1;
													fix_idx = 1;
													//reinisiasi arrDG
													for (int i = 1; i <= 10; i++) {
															m[i].arrDG[0] = fixm[i].arrDG[0];
															m[i].arrDG[1] = fixm[i].arrDG[1];
															m[i].arrDG[2] = fixm[i].arrDG[2];
															m[i].arrDG[3] = fixm[i].arrDG[3];
															m[i].currDG = fixm[i].currDG;
													}

													eraseDisplay();
													stringFormat(intTostr, "%.2d", idx);
													displayTextLine(9, "Simpangan ke: %s", intTostr);

											}
									}

							} //endif gtn=-1
					} //endwhilenextline

					if (found_end == 1) {

							// bersiap-siap kembali ke kotak start setelah sampai di kotak finish
							eraseDisplay();
							displayTextLine(3, "Bersiap kembali ke kotak Start");
							wait1Msec(1500);

							goback();

							BACK_FOUND = 0;
							BACK_BLUE = 0;

							while (idx >= 1) {
									eraseDisplay();
									if (BACK_FOUND == 0) {
											BACK_FOUND = 1;
											stringFormat(intTostr, "%.2d", idx);
											displayTextLine(3, "Jalur antara:");
											displayTextLine(4, "finish");
											displayTextLine(5, "%s", intTostr);
									} else {
											stringFormat(intTostr, "%.2d", TEMP_IDX);
											displayTextLine(3, "Jalur antara:");
											displayTextLine(4, "%s", intTostr);
											stringFormat(intTostr, "%.2d", m[TEMP_IDX].connectedWith);
											displayTextLine(5, "%s", intTostr);
									}
									wait1Msec(2000);
									TEMP_IDX = idx;
									gotofirst(m[idx], 0, BACK_BLUE);
									idx = m[idx].connectedWith;
							}

							eraseDisplay();
							displayTextLine(3, "Jalur antara:");
							displayTextLine(4, "1");
							displayTextLine(5, "start");
							wait1Msec(2000);

							state = LineFollow(counterBlue);

							eraseDisplay();
							displayTextLine(12, "Berhasil sampai ke start");
							displayTextLine(13, "Kembali ke finish lagi");
							wait1Msec(1500);

							goback();

							BACK_FOUND = 0;
							BACK_BLUE = 0;
							idx = 1;
							while (BACK_BLUE == 0) {
									eraseDisplay();
									if (BACK_FOUND == 0) {
											BACK_FOUND = 1;
											stringFormat(intTostr, "%.2d", idx);
											displayTextLine(3, "Jalur antara:");
											displayTextLine(4, "start");
											displayTextLine(5, "%s", intTostr);
									} else {
											stringFormat(intTostr, "%.2d", TEMP_IDX);
											displayTextLine(3, "Jalur antara:");
											displayTextLine(4, "%s", intTostr);
											stringFormat(intTostr, "%.2d", m[TEMP_IDX].friend[m[TEMP_IDX].indexJalur]);
											displayTextLine(5, "%s", intTostr);
									}
									wait1Msec(2000);
									TEMP_IDX = idx;
									gotofirst(m[idx], 1, BACK_BLUE);
									idx = m[idx].friend[m[idx].indexJalur];
									if (idx == -1) {
											break;
									}
							}

							eraseDisplay();
							displayTextLine(3, "Jalur antara:");
							stringFormat(intTostr, "%.2d", TEMP_IDX);
							displayTextLine(4, "%s", intTostr);
							displayTextLine(5, "finish");
							wait1Msec(2000);
							state = LineFollow(counterBlue);
							break;

					} else { //found_end = 0
							break;
					}

			} else {
					//state = 3 //kemungkinan tidak akan masuk ke sini
					displayTextLine(13, "FINALLY RED");
			}
	}	//endwhile

}