package vn.deeplearning;
import edu.princeton.cs.algs4.StdIn;
import edu.princeton.cs.algs4.StdOut;

public class QuickFindUF {

    private int[] id; // id[i] = component identifier of i
    private int count; // number of components

    public QuickFindUF(int N){
        count = N;
        id = new int[N];
        for (int i = 0; i < N; i++){
            id[i] = i;
        }
    }

    public boolean connected(int p, int q){
        return id[p] == id[q];
    }

    public void union(int p, int q){
        int pid = id[p];
        int qid = id[q];

        if(pid == qid) return;

        for(int i = 0; i < id.length ; i++){
            if(id[i] == pid){
                id[i] = qid;
            }
        }
        count --;
    }

    public int count(){
        return count;
    }

    public static void main(String[] args) {
        StdOut.println("Enter n");
        int n = StdIn.readInt();

        QuickFindUF uf = new QuickFindUF(n);
        while (!StdIn.isEmpty()){
            StdOut.println("Enter p, q");
            int p = StdIn.readInt();
            int q = StdIn.readInt();

            if(p < 0){
                break;
            }

            if (uf.connected(p, q)) continue;
            uf.union(p, q);
            StdOut.println(p + " " + q);

        }
        StdOut.println(uf.count() + " components");
    }
}
