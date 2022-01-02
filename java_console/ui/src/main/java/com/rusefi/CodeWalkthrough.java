package com.rusefi;

import com.devexperts.logging.Logging;
import com.rusefi.livedata.ParseResult;
import com.rusefi.livedata.generated.CPP14Parser;
import com.rusefi.livedata.generated.CPP14ParserBaseListener;
import com.rusefi.ui.livedata.Range;
import com.rusefi.ui.livedata.SourceCodePainter;
import com.rusefi.ui.livedata.VariableValueSource;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.Token;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import org.antlr.v4.runtime.tree.TerminalNode;
import org.jetbrains.annotations.NotNull;

import java.awt.*;
import java.util.ArrayList;
import java.util.Stack;
import java.util.concurrent.atomic.AtomicReference;

import static com.devexperts.logging.Logging.getLogging;

public class CodeWalkthrough {
    private static final Logging log = getLogging(CodeWalkthrough.class);
    // inactive statements within inactive branch of code - light red
    public static final Color INACTIVE_BRANCH = new Color(255, 102, 102);
    // active statements - light green
    public static final Color ACTIVE_STATEMENT = new Color(102, 255, 102);
    // cost past active return statement
    public static final Color PASSIVE_CODE = Color.lightGray;
    public static final Color BROKEN_CODE = Color.orange;
    public static final Color TRUE_CONDITION = Color.GREEN;
    public static final Color FALSE_CONDITION = Color.RED;

    static {
        log.configureDebugEnabled(false);
    }

    private static final String CONFIG_MAGIC_PREFIX = "engineConfiguration";

    public static ParseResult applyVariables(VariableValueSource valueSource, String sourceCode, SourceCodePainter painter, ParseTree tree) {
        Stack<BranchingState> currentState = new Stack<>();
        java.util.List<String> brokenConditions = new ArrayList<>();

        java.util.List<TerminalNode> allTerminals = new java.util.ArrayList<>();

        new ParseTreeWalker().walk(new CPP14ParserBaseListener() {
            @Override
            public void enterFunctionDefinition(CPP14Parser.FunctionDefinitionContext ctx) {
                // new method is starting new all over
                resetState(currentState);
            }

            @Override
            public void enterDeclarationStatement(CPP14Parser.DeclarationStatementContext ctx) {
                super.enterDeclarationStatement(ctx);
                colorStatement(ctx, painter);
            }

            @Override
            public void enterExpressionStatement(CPP14Parser.ExpressionStatementContext ctx) {
                super.enterExpressionStatement(ctx);
                colorStatement(ctx, painter);
            }

            @Override
            public void enterJumpStatement(CPP14Parser.JumpStatementContext ctx) {
                super.enterJumpStatement(ctx);
                colorStatement(ctx, painter);
                if ("return".equalsIgnoreCase(ctx.getStart().getText()) &&
                        !currentState.isEmpty() &&
                        getOverallState(currentState) == BranchingState.TRUE) {
                    // we have experienced 'return' in 'green' active flow looks like end of execution for this method?
                    currentState.clear();
                }
            }

            @Override
            public void visitTerminal(TerminalNode node) {
                allTerminals.add(node);
                if ("else".equalsIgnoreCase(node.getSymbol().getText())) {
                    if (log.debugEnabled())
                        log.debug("CONDITIONAL ELSE terminal, flipping condition");

                    if (currentState.isEmpty())
                        return;

                    BranchingState onTop = currentState.pop();
                    currentState.add(onTop.flip());
                }

            }

            @Override
            public void enterCondition(CPP14Parser.ConditionContext ctx) {
                String conditionVariable = ctx.getText();
                if (log.debugEnabled())
                    log.debug("CONDITIONAL: REQUESTING VALUE " + conditionVariable);

                Boolean state = (Boolean) valueSource.getValue(conditionVariable);
                BranchingState branchingState = BranchingState.valueOf(state);
                if (log.debugEnabled())
                    log.debug("CURRENT STATE ADD " + state);
                currentState.add(branchingState);
                if (branchingState == BranchingState.BROKEN) {
                    brokenConditions.add(conditionVariable);
                    painter.paintBackground(BROKEN_CODE, new Range(ctx));
                } else if (branchingState == BranchingState.TRUE) {
                    painter.paintBackground(TRUE_CONDITION, new Range(ctx));
                } else {
                    painter.paintBackground(FALSE_CONDITION, new Range(ctx));
                }
            }

            @Override
            public void exitSelectionStatement(CPP14Parser.SelectionStatementContext ctx) {
                super.exitSelectionStatement(ctx);
                if (currentState.isEmpty())
                    return; // we are here if some conditional variables were not resolved
                BranchingState onTop = currentState.pop();
                if (onTop == BranchingState.BROKEN)
                    currentState.push(BranchingState.BROKEN);
                if (log.debugEnabled())
                    log.debug("CONDITIONAL: EXIT");
            }


            private void colorStatement(ParserRuleContext ctx, SourceCodePainter painter) {
                Color color;
                if (currentState.isEmpty()) {
                    color = PASSIVE_CODE; // we are past return or past error
                } else {
                    BranchingState isAlive = getOverallState(currentState);
                    if (isAlive == BranchingState.BROKEN) {
                        color = BROKEN_CODE;
                    } else {
                        color = isAlive == BranchingState.TRUE ? ACTIVE_STATEMENT : INACTIVE_BRANCH;
                    }
                }
                Range range = new Range(ctx);
                if (log.debugEnabled())
                    log.info(color + " for " + sourceCode.substring(range.getStart(), range.getStop()));
                painter.paintBackground(color, range);
            }
        }, tree);

        java.util.List<Token> configTokens = new java.util.ArrayList<>();

        for (int i = 0; i < allTerminals.size() - 3; i++) {

            if (allTerminals.get(i).getText().equals(CONFIG_MAGIC_PREFIX) &&
                    allTerminals.get(i + 1).getText().equals("->")
            ) {
                Token token = allTerminals.get(i + 2).getSymbol();
                painter.paintForeground(Color.BLUE, new Range(token, token));
                configTokens.add(token);
            }
        }
        return new ParseResult(configTokens, brokenConditions);
    }

    private static void resetState(Stack<BranchingState> currentState) {
        currentState.clear();
        currentState.add(BranchingState.TRUE);
    }

    private static BranchingState getOverallState(Stack<BranchingState> currentState) {
        for (BranchingState value : currentState) {
            if (value == BranchingState.BROKEN)
                return BranchingState.BROKEN;
        }
        for (BranchingState value : currentState) {
            if (value == BranchingState.FALSE)
                return BranchingState.FALSE;
        }
        return BranchingState.TRUE;
    }

    @NotNull
    private static String getOrigin(ParserRuleContext ctx, String s) {
        Range range = new Range(ctx);
        return s.substring(range.getStart(), range.getStop());
    }

    private enum BranchingState {
        TRUE,
        FALSE,
        BROKEN;

        public static BranchingState valueOf(Boolean state) {
            if (state == null)
                return BROKEN;
            return state ? TRUE : FALSE;
        }

        public BranchingState flip() {
            switch (this) {
                case TRUE:
                    return FALSE;
                case FALSE:
                    return TRUE;
                case BROKEN:
                default:
                    return BROKEN;
            }
        }
    }
}
