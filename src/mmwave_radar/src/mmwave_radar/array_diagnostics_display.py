import matplotlib.pyplot as plt

from radar_messages.msg import StampedReport

from mmwave_radar.types import RadarFrame



class ArrayDiagnosticsDisplay(object):
    def __init__(self, radar_count):
        self.radar_count = radar_count

        self.fig, self.axs = plt.subplots(self.radar_count, 1,
                                          figsize=(10,10),
                                          facecolor='black',
                                          squeeze=False)
        
    def plot_live_radars(self, radar_msgs):
        assert len(radar_msgs) == self.radar_count

        sorted_msgs = sorted(radar_msgs, key=lambda x: x.header.frame_id)
        for i, m in enumerate(sorted_msgs):
            ignore_before = 2
            interesting_gates = m.gate_energies[ignore_before:]

            title = f"{m.header.frame_id} Gate Intensities"
            labels = range(ignore_before, len(m.gate_energies))
            colours = ['tab:cyan'] * len(interesting_gates)
            
            # print(f"# values : {len(interesting_gates)}")
            # print(f"# labels : {len(labels)}")
            # print(f"# colours : {len(colours)}")

            self._plot_bar(self.axs[i, 0], title, "Gates", "Magnitudes (units)", 
                           labels,
                           colours,
                           interesting_gates)
            
        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)

    def _plot_bar(self, ax, title, x_label, y_label, labels, colours, values):
        ax.clear()
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
        

