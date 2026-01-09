#pragma once

class PID {
public:
	PID(double kp = 0, double ki = 0, double kd = 0,
			double outMin = -1e9, double outMax = 1e9,
			double iMin = -1e9, double iMax = 1e9)
			: m_kp(kp), m_ki(ki), m_kd(kd), m_outMin(outMin), m_outMax(outMax),
				m_iMin(iMin), m_iMax(iMax) {}

	void setTunings(double kp, double ki, double kd) {
		m_kp = kp; m_ki = ki; m_kd = kd;
	}
	void setOutputLimits(double mn, double mx) { m_outMin = mn; m_outMax = mx; }
	void setIntegralLimits(double mn, double mx) { m_iMin = mn; m_iMax = mx; }

	void reset() { m_prevErr = 0; m_integral = 0; m_first = true; }

	double step(double error, double dt) {
		if (dt <= 0) dt = 1e-3;
		if (m_first) { m_prevErr = error; m_first = false; }
		m_integral += error * dt;
		if (m_integral > m_iMax) m_integral = m_iMax;
		if (m_integral < m_iMin) m_integral = m_iMin;
		double deriv = (error - m_prevErr) / dt;
		m_prevErr = error;
		double out = m_kp * error + m_ki * m_integral + m_kd * deriv;
		if (out > m_outMax) out = m_outMax;
		if (out < m_outMin) out = m_outMin;
		return out;
	}

private:
	double m_kp, m_ki, m_kd;
	double m_outMin, m_outMax;
	double m_iMin, m_iMax;
	double m_prevErr = 0;
	double m_integral = 0;
	bool m_first = true;
};

