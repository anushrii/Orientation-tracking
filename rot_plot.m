function rot_plot(ukf,rots,frame_no)

    subplot(1,2,1),rotplot(ukf)
    title(['ukf ',num2str(frame_no)])
    subplot(1,2,2),rotplot(rots)
   title('Vicon')
end