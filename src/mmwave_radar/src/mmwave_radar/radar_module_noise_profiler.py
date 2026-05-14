from mmwave_radar.types import SixteenGateReport

class RadarModuleNoiseProfiler(object):
    def __init__(self, profile_path):
        self.running_reports = {}

        self.profile_path = profile_path
        pass

    def add_report(self, frame, new_report: SixteenGateReport):
        # self.recorded_reports.append(new_report)
        self._update_running(self.running_reports,
                             label=frame,
                             value=new_report,
                             colour='tab:orange')
        
        self.store_profile()

    def store_profile(self):
        with open(self.profile_path, 'w') as f:
            for label in self.running_reports:
                
                reports = self.running_reports[label]['data']

                n = len(reports)
                totals = [0] * 16 
                for gR in reports:
                    for i, g in enumerate(gR.gates):
                        totals[i] += g
                averages = [g / n for g in totals]

                buffer = f"{label} -> {str(averages)}\n"
                f.write(buffer)

    def _update_running(self, runningValueSet:dict, label, value, colour):
        if label not in runningValueSet:
            runningValueSet[label] = {'colour': colour,
                                      'data': [value],
                                      }# 'time': [time]
        else:
            # runningValueSet[label]['time'].append(time)
            runningValueSet[label]['data'].append(value)