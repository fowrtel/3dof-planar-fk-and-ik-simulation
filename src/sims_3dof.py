import numpy as np
import matplotlib.pyplot as plt

# --- FUNGSI PLOT ROBOT ---
def plot_robot(L1, L2, L3, th1_deg, th2_deg, th3_deg, target_x=None, target_y=None):
    # Konversi ke radian
    th1 = np.radians(th1_deg)
    th2 = np.radians(th2_deg)
    th3 = np.radians(th3_deg)

    # Posisi pangkal (Base)
    x0, y0 = 0, 0

    # Posisi Sendi 1
    x1 = L1 * np.cos(th1)
    y1 = L1 * np.sin(th1)

    # Posisi Sendi 2
    x2 = x1 + L2 * np.cos(th1 + th2)
    y2 = y1 + L2 * np.sin(th1 + th2)

    # Posisi Ujung (End-Effector)
    x3 = x2 + L3 * np.cos(th1 + th2 + th3)
    y3 = y2 + L3 * np.sin(th1 + th2 + th3)

    # Kumpulkan titik X dan Y untuk digambar
    X = [x0, x1, x2, x3]
    Y = [y0, y1, y2, y3]

    # Setup Plot
    plt.figure(figsize=(6, 6))
    plt.title("Simulasi Lengan Robot 3-DOF Planar")
    
    # Gambar garis lengan dan titik sendi
    plt.plot(X, Y, '-o', color='blue', linewidth=4, markersize=8, label='Lengan Robot')
    plt.plot(x0, y0, 's', color='black', markersize=10, label='Base (0,0)') # Pangkal
    
    # Jika mode IK, gambar target posisinya
    if target_x is not None and target_y is not None:
        plt.plot(target_x, target_y, '*', color='red', markersize=15, label='Target')

    # Pengaturan sumbu agar skalanya proporsional
    max_reach = L1 + L2 + L3 + 10
    plt.xlim(-max_reach, max_reach)
    plt.ylim(-max_reach, max_reach)
    
    plt.axhline(0, color='gray', linestyle='--', linewidth=1)
    plt.axvline(0, color='gray', linestyle='--', linewidth=1)
    plt.grid(True)
    plt.legend()
    plt.xlabel("Sumbu X")
    plt.ylabel("Sumbu Y")
    
    # Tampilkan window plot
    print(">> Menampilkan grafik... (Tutup jendela grafik untuk melanjutkan)")
    plt.show()

# --- FUNGSI FORWARD KINEMATICS ---
def forward_kinematics(L1, L2, L3, th1_deg, th2_deg, th3_deg):
    th1 = np.radians(th1_deg)
    th2 = np.radians(th2_deg)
    th3 = np.radians(th3_deg)

    x = L1 * np.cos(th1) + L2 * np.cos(th1 + th2) + L3 * np.cos(th1 + th2 + th3)
    y = L1 * np.sin(th1) + L2 * np.sin(th1 + th2) + L3 * np.sin(th1 + th2 + th3)
    phi_deg = th1_deg + th2_deg + th3_deg

    return x, y, phi_deg

# --- FUNGSI INVERSE KINEMATICS ---
def inverse_kinematics(L1, L2, L3, x, y, phi_deg):
    phi = np.radians(phi_deg)

    # 1. Cari Wrist point
    xw = x - L3 * np.cos(phi)
    yw = y - L3 * np.sin(phi)

    # 2. Hitung theta 2
    C2 = (xw**2 + yw**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if C2 < -1.0 or C2 > 1.0:
        return None, "Target di luar jangkauan lengan robot!"

    S2 = np.sqrt(1 - C2**2) # Ambil solusi elbow-down
    th2 = np.arctan2(S2, C2)

    # 3. Hitung theta 1
    k1 = L1 + L2 * np.cos(th2)
    k2 = L2 * np.sin(th2)
    th1 = np.arctan2(yw, xw) - np.arctan2(k2, k1)

    # 4. Hitung theta 3
    th3 = phi - th1 - th2

    return (np.degrees(th1), np.degrees(th2), np.degrees(th3)), "Sukses"

# --- PROGRAM UTAMA ---
def main():
    print("=== Simulator Kinematika 3-DOF Planar (Numpy & Matplotlib) ===")
    
    L1 = float(input("Masukkan panjang Link 1: "))
    L2 = float(input("Masukkan panjang Link 2: "))
    L3 = float(input("Masukkan panjang Link 3: "))

    while True:
        print("\nPilih Mode:")
        print("1. Forward Kinematics (Hitung Posisi & Gambar)")
        print("2. Inverse Kinematics (Hitung Sudut & Gambar)")
        print("3. Keluar")
        
        pilihan = input("Pilihan (1/2/3): ")

        if pilihan == '1':
            t1 = float(input("Theta 1 (derajat): "))
            t2 = float(input("Theta 2 (derajat): "))
            t3 = float(input("Theta 3 (derajat): "))
            
            x, y, phi = forward_kinematics(L1, L2, L3, t1, t2, t3)
            print(f"\n=> Posisi End-Effector: X={x:.2f}, Y={y:.2f}, Phi={phi:.2f} deg")
            
            # Gambar visualisasinya
            plot_robot(L1, L2, L3, t1, t2, t3)

        elif pilihan == '2':
            x = float(input("Target X: "))
            y = float(input("Target Y: "))
            phi = float(input("Target Orientasi Phi (derajat): "))
            
            hasil, status = inverse_kinematics(L1, L2, L3, x, y, phi)
            
            if hasil:
                t1, t2, t3 = hasil
                print(f"\n=> Sudut Ditemukan: Theta1={t1:.2f}, Theta2={t2:.2f}, Theta3={t3:.2f}")
                # Gambar visualisasinya dengan penanda target
                plot_robot(L1, L2, L3, t1, t2, t3, target_x=x, target_y=y)
            else:
                print(f"\n[!] Error: {status}")
            
        elif pilihan == '3':
            print("Keluar dari program. Terima kasih!")
            break
        else:
            print("Pilihan tidak valid.")

if __name__ == "__main__":
    main()