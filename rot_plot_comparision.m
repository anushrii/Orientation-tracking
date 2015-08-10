function rot_plot_comparision(ukf,Ra,Rw, frame_no)

    subplot(1,3,1),rotplot(ukf)
    title(['ukf ',num2str(frame_no)])
    subplot(1,3,2),rotplot(Rw)
    title('gyro ')
    subplot(1,3,3),rotplot(Ra)
    title('accelerometer')
end