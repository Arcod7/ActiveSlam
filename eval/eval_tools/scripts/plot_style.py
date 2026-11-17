"""One place for the figure style: evo's look (seaborn whitegrid, deep palette).

Every plotting script calls apply() right after selecting the Agg backend, so
the dissertation figures match the evo trajectory plots they sit next to.
"""
import matplotlib

# evo's arm colours stay stable across every figure, whatever subset is drawn.
ARM_COLOR = {
    'no_lc': '#7f7f7f',
    'lc': '#1f77b4',
    'lc_revisit': '#d62728',
    'lc_revisit_20m': '#2ca02c',
}


def apply():
    try:
        import seaborn as sns
        sns.set_theme(style='whitegrid', palette='deep', context='notebook')
    except ImportError:
        matplotlib.rcParams['axes.grid'] = True
        matplotlib.rcParams['grid.alpha'] = 0.4
    matplotlib.rcParams.update({
        'figure.dpi': 100,
        'savefig.dpi': 150,
        'axes.titlesize': 11,
        'axes.labelsize': 10,
        'legend.fontsize': 8,
        'lines.linewidth': 1.5,
    })
