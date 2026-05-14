import matplotlib.pyplot as plt

from radar_messages.msg import StampedReport
from mmwave_radar.radar_module_noise_profiler import RadarModuleNoiseProfiler
from mmwave_radar.types import SixteenGateReport, GateReportQueue 

from mmwave_radar.noise_profile_library import RadarNoise

class ArrayDiagnosticsDisplay(object):
    def __init__(self, radar_count):
        self.radar_count = radar_count

        self.queue_report_window = 10
        self.report_queues = [GateReportQueue(self.queue_report_window)
                      for _ in range(self.radar_count)]

        self.noise_profile = RadarModuleNoiseProfiler('solo_45.txt')

        self.fig, self.axs = plt.subplots(self.radar_count, 3,
                                          figsize=(10,10),
                                          facecolor='black',
                                          squeeze=False)

    def routine(self, radar_msgs, t):
        sorted_msgs = sorted(radar_msgs, key=lambda x: x.header.frame_id)

        for i, m in enumerate(sorted_msgs):
            formatted = SixteenGateReport.from_stamped_report(m)
            self.report_queues[i].push(formatted)

            if self.report_queues[i].name == 'none':
                self.report_queues[i].set_name(m.header.frame_id)
            
            self.noise_profile.add_report(self.report_queues[i].name, formatted)

        self.plot_radar_queues()

        
    def plot_radar_queues(self):

        # plotting
        for i, rQ in enumerate(self.report_queues):
            live = rQ.top

            if rQ.name == 'radar0':
                noise = RadarNoise.frame_zero_full_45_0
            elif rQ.name == 'radar1':
                noise = RadarNoise.frame_zero_full_45_1
            elif rQ.name == 'radar2':
                noise = RadarNoise.frame_zero_full_45_2
            else:
                print('name mismatch -> ', rQ.name)
                raise RuntimeError("Done fucked up")

            denoised_mean = (rQ.avg - noise)

            labels = range(len(live.gates))
            colours = ['tab:cyan'] * len(live.gates)
            
            # print(f"# values : {len(interesting_gates)}")
            # print(f"# labels : {len(labels)}")
            # print(f"# colours : {len(colours)}")

            
            title_live = f"{rQ.name} Gate Intensities - Live"
            self._plot_bar(self.axs[i, 0], title_live, "Gates", "Magnitudes (units)", 
                           labels,
                           colours,
                           live.gates)

            
            title_sum = f"{rQ.name} Denoised Mean"
            
            self._plot_bar(self.axs[i, 1], title_sum, "Gates", "Magnitudes (units)", 
                           labels,
                           colours,
                           denoised_mean.gates,
                           max_val=10000)
            
            title_sum = f"{rQ.name} Denoised Mean -> Distance Guess"
            self._plot_bar(self.axs[i, 2], title_sum, "Gates", "Magnitudes (units)", 
                           ['Estimate'],
                           'tab:orange',
                           [denoised_mean.dist_est()],
                           max_val=2.5)
            
        plt.tight_layout()
        plt.draw()
        plt.pause(0.00001)


    def _plot_bar(self, ax, title, x_label, y_label, labels, colours, values, max_val=-1.0):
        ax.clear()

        if max_val != -1:
            ax.set_ylim(0,max_val)

        ax.set_facecolor('darkgrey')
        # ax.set_aspect("equal")
        ax.set_xlabel(x_label, fontweight='bold', color='white')
        ax.set_ylabel(y_label, fontweight='bold', color='white')
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.set_title(title, fontweight='bold', color='white')

        ax.bar(labels, values, color=colours)

        ax.set_xticks(labels)
        ax.set_xticklabels(labels, fontweight='bold', color='white',
                           rotation=45, ha='right')

        ticks_loc = ax.get_yticks()
        ax.set_yticks(ticks_loc)
        ax.set_yticklabels(ticks_loc, fontweight='bold', color='white')

    def _setup_line_plots(self, ax, title, x_label, y_label):
        ax.clear()
        ax.set_facecolor('darkgrey')
        # ax.set_aspect("equal")
        ax.set_xlabel(x_label, fontweight='bold', color='white')
        ax.set_ylabel(y_label, fontweight='bold', color='white')
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.set_title(title, fontweight='bold', color='white')

        ax.tick_params(axis='y', color='white', labelcolor='white')
        for label in ax.get_yticklabels():
            label.set_fontweight('bold')
        # ax.set_yticklabels(ax.get_yticks(), fontweight='bold', colour='white')

    def plot_line(self, ax, x_data , y_data, color):
        ax.plot(x_data, y_data, color=color)

    def _update_running(self, runningValueSet:dict, label, value, time, colour):
        if label not in runningValueSet:
            runningValueSet[label] = {'colour': colour,
                                      'data': [value],
                                      'time': [time]}
        else:
            runningValueSet[label]['time'].append(time)
            runningValueSet[label]['data'].append(value)
        

