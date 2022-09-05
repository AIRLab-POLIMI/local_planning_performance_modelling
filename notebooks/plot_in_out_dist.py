import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from os import path

def plot_in_out_dist(x_metric, y_metric, x_bin_width, y_bin_width, selected_df, x_lim=None, y_lim=None, out_dir=None):

    df_notna = selected_df[
        selected_df[x_metric.name].notna() & 
        selected_df[y_metric.name].notna()
    ].copy()

    if len(df_notna[x_metric.name]) == 0:
        print(f"len(df_notna[{x_metric.name}]) == 0")
    else:

        if x_bin_width:
            bins_x = np.arange(0., x_metric.max() + x_bin_width, x_bin_width)
            df_notna['x_range'] = pd.cut(df_notna[x_metric.name], bins_x, labels=bins_x[:-1]).astype('float')
        else:
            bins_x = sorted(x_metric.unique())
            df_notna['x_range'] = df_notna[x_metric.name]

        if y_bin_width:
            bins_y = np.arange(0., y_metric.max() + y_bin_width, y_bin_width)
            df_notna['y_range'] = pd.cut(df_notna[y_metric.name], bins_y, labels=bins_y[:-1]).astype('float')
        else:
            bins_y = sorted(y_metric.unique())
            df_notna['y_range'] = df_notna[y_metric.name]

        df_g = df_notna.groupby(['x_range', 'y_range']).count()[y_metric.name]

        s = df_g.sum()
        m = np.zeros((len(bins_x), len(bins_y)))
        for i_x, x in enumerate(bins_x):
            for i_y, y in enumerate(bins_y):
                m[i_x, i_y] = df_g[(x, y)]/s if (x, y) in df_g.index else 0

        if f"{np.sum(m):g}" != f"{1.0:g}":
            print_warn(f"np.sum(m) == {np.sum(m)} != 1.0")

        fig = plt.figure()
        y_quota = np.max(bins_y)/y_lim[1] if y_lim else 1
        x_quota = np.max(bins_x)/x_lim[1] if x_lim else 1
        plt.imshow(m.T, origin='lower', aspect=len(bins_x)/len(bins_y)*y_quota/x_quota)

        x_ticks_pos = np.arange(len(bins_x)) - 0.5*(x_bin_width is not None)
        plt.xticks(x_ticks_pos, map(lambda x: f"{x:g}", bins_x), rotation=-90)
        y_ticks_pos = np.arange(len(bins_y)) - 0.5*(y_bin_width is not None)
        plt.yticks(y_ticks_pos, map(lambda y: f"{y:g}", bins_y))
        plt.xlabel(x_metric.name.replace('_', ' '))
        plt.ylabel(y_metric.name.replace('_', ' '))

        if y_lim:
            plt.ylim(np.array(y_lim)/y_bin_width)
        if x_lim:
            plt.xlim(np.array(x_lim)/x_bin_width)
        fig.set_size_inches([4, 4])
        if out_dir:
            if not path.exists(out_dir):
                os.makedirs(out_dir)
            fig.savefig(out_dir + f"/{y_metric.name}---{x_metric.name}--joint-distribution.pdf".replace('_', '-'), bbox_inches='tight')
        plt.show()
        plt.close()
