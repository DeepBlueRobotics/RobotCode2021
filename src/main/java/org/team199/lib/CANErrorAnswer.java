package org.team199.lib;

import com.revrobotics.CANError;

import org.mockito.internal.stubbing.defaultanswers.ReturnsSmartNulls;
import org.mockito.invocation.InvocationOnMock;

/**
 * Represents a {@link org.mockito.Mockito} {@link org.mockito.stubbing.Answer} which returns {@link CANError#kOk}
 * for methods which return an {@link CANError} object and refers all other calls to {@link ReturnsSmartNulls}
 */
public class CANErrorAnswer extends ReturnsSmartNulls {

    private static final long serialVersionUID = -561160298532167923L;

    @Override
    public Object answer(InvocationOnMock invocation) throws Throwable {
        return invocation.getMethod().getReturnType().equals(CANError.class) ? CANError.kOk : super.answer(invocation);
    }
    
}