function [M_qA_qB] = generate_M(QAi,QBi)

SA          = QAi(1);
VA          = QAi(2:4);
SB          = QBi(1);
VB          = QBi(2:4);
w           = VA + VB;
sk_AB     = skew_sym(w);

M_qA_qB     = [SA-SB,  -(VA-VB)';
              (VA-VB), (SA-SB)*eye(3)+sk_AB];

end