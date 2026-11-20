# Canintech_Exercicios

Resoluções das atividades do curso de programação FRC (simulação com AdvantageScope, robô tanque).

| Pasta | Atividade | O que foi feito |
| --- | --- | --- |
| [Atividade1_Aula17](Atividade1_Aula17) | Aula 17 – Atividade 1 | Auto-hand (coleta automática para o braço), coleta protegida pelo intake e proteção do braço com o elevador embaixo |
| [Atividade2_Aula19](Atividade2_Aula19) | Aula 19 – Atividade 2 | Autônomo "3 Algas" (2 na net + 1 no processor), botão X com drive to pose (7,8; 4,0; 60°) e seletor de autônomo no Elastic |

## Como rodar

1. Abra a pasta da atividade no WPILib VS Code (2026).
2. Rode **WPILib: Simulate Robot Code**.
3. Conecte o AdvantageScope em modo simulação.
4. Atividade 2: no Elastic, use **IP Address Mode = Localhost** e **File → Load Layout from Robot**.

## Atividade 1 – principais trechos (`RobotContainer.java`)

- `coletar_Ground()`: o intake só coleta se não houver alga dentro do robô e o elevador estiver ≥ 200.
- `autoHand()`: com alga dentro do robô e braço vazio, sobe o elevador, centraliza o braço em 0°, desce e liga os roletes por 1 s.
- `moverBracoProtegido()`: os botões do braço só giram com o elevador acima de 390 (melhoria extra além da solução oficial).

## Atividade 2 – principais trechos

- `Autos.tresAlgas(...)`: reset de odometria em (7,13; 1,18), coleta das três algas contornando o reef e pontuação com os blocos `prepararColeta`, `coletar` e `pontuar`.
- `RobotContainer`: opção "3 Algas" no seletor e `driver.x()` → `tank.driveToPose(7.8, 4.0, 60)`.
- `Robot.java` + `src/main/deploy/elastic-layout.json`: layout do Elastic servido na porta 5800.
