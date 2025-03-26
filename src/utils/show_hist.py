import matplotlib.pyplot as plt


def show_hist(hist, bin_edges):
    plt.figure(figsize=(10, 6))
    plt.bar(bin_edges[:-1], hist, width=bin_edges[1]-bin_edges[0], align='edge')
    plt.xlabel('Z座標 (m)')
    plt.ylabel('頻度')
    plt.title('点群のZ座標分布')
    plt.grid(True, alpha=0.3)
    plt.show()
    