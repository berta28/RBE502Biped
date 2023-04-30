import matplotlib.pyplot as plt
import pandas as pd
import csv



def main():
    df = pd.read_csv('vg_controller_demo/results/sim_results_1682870286.csv')
    col_headers = list(df.columns)

    plt.figure('Lowerleg')
  
    plt.plot(df[col_headers[0]], df[col_headers[1]], label=col_headers[1])
    plt.plot(df[col_headers[0]], df[col_headers[3]], label=col_headers[3])
    plt.plot(df[col_headers[0]], df[col_headers[2]], label=col_headers[2])
    plt.plot(df[col_headers[0]], df[col_headers[4]], label=col_headers[4])
    plt.title('Torque vs Time')
    plt.xlabel('Time [s]')
    plt.xlim([0,37])
    plt.ylabel('Torque N*cm')
    plt.ylim([-80,80])
    plt.legend()
    # plt.show()
    plt.savefig('vg_controller_demo/results/torques.png')


    plt.figure('XY')
  
    plt.plot(df[col_headers[0]], df[col_headers[5]], label=col_headers[5])
    plt.plot(df[col_headers[0]], df[col_headers[6]], label=col_headers[6])
    plt.title('Position vs Time')
    plt.xlabel('Time [s]')
    plt.xlim([0,37])
    plt.ylabel('Position [mm]')
    plt.ylim([-3,222])
    plt.legend()
    # plt.show()000000000000
    plt.savefig('vg_controller_demo/results/position.png')

    plt.figure('XY_error')
  
    plt.plot(df[col_headers[0]], df[col_headers[5]]-0, label=col_headers[5])
    plt.plot(df[col_headers[0]], df[col_headers[6]]-219.48, label=col_headers[6])
    plt.title('Position error vs Time')
    plt.xlabel('Time [s]')
    plt.xlim([0,37])
    plt.ylabel('Position [mm]')
    plt.ylim([-2.5,1.5])
    plt.legend()
    # plt.show()
    plt.savefig('vg_controller_demo/results/position_error.png')


    plt.figure('Force vs Time')
  
    plt.plot(df[col_headers[0]], df[col_headers[7]], label=col_headers[7])
    plt.title('Force vs Time')
    plt.xlabel('Time [s]')
    plt.xlim([0,37])
    plt.ylabel('Force [N]')
    plt.ylim([-2.5,6])
    plt.legend()
    plt.show()
    plt.savefig('vg_controller_demo/results/force.png')
    
if __name__ == "__main__":
    main()